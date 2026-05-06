#include "battery_modbus.h"

#include "app_config.h"
#include "app_diagnostics.h"
#include "battery_data.h"
#include "board_io.h"
#include "cmsis_os.h"
#include "modbuc.h"
#include "slave_update_thread.h"
#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static const uint16_t modbusStartRegister = 0u;
static ModbusChannelBuffer_t modbusBuffers[UART_CHANNEL_COUNT] = {0};

#define BATTERY_MODBUS_POLL_SLOT_MS          (100u)
#define BATTERY_MODBUS_RESPONSE_TIMEOUT_MS   (100u)
#define BATTERY_MODBUS_MAX_RETRIES           (3u)
#define BATTERY_MODBUS_TX_TIMEOUT_MS         (20u)

#define BATTERY_MODBUS_ERROR_FLAG            (0x80000000u)
#define BATTERY_MODBUS_ERROR_TIMEOUT         (0x00010000u)
#define BATTERY_MODBUS_ERROR_RESPONSE        (0x00020000u)
#define BATTERY_MODBUS_ERROR_UART            (0x00030000u)
#define BATTERY_MODBUS_ERROR_TX              (0x00040000u)
#define BATTERY_MODBUS_ERROR_MUTEX           (0x00050000u)
#define BATTERY_MODBUS_ERROR_RETRY_LIMIT     (0x00060000u)

typedef enum
{
    BATTERY_MODBUS_FRAME_NONE = 0,
    BATTERY_MODBUS_FRAME_BOOTLOADER,
    BATTERY_MODBUS_FRAME_VALID,
    BATTERY_MODBUS_FRAME_INVALID
} BatteryModbusFrameResult_t;

static uint8_t modbusNextChannel = 0u;
static uint8_t modbusActiveChannel = 0u;
static uint8_t modbusRetryCount = 0u;
static uint32_t modbusRequestTick = 0u;
static uint32_t modbusNextPollTick = 0u;
static bool modbusAwaitingResponse = false;

static uint8_t BatteryModbus_NextChannel(uint8_t channelIndex)
{
    return (uint8_t)((channelIndex + 1u) % UART_CHANNEL_COUNT);
}

static bool BatteryModbus_TimeReached(uint32_t currentTick, uint32_t targetTick)
{
    return ((int32_t)(currentTick - targetTick) >= 0);
}

static void BatteryModbus_LogError(uint8_t channelIndex, uint32_t reason, uint32_t detail)
{
    if (channelIndex >= UART_CHANNEL_COUNT)
    {
        return;
    }

    lastProcessedUartNumber = uartIndexToNumber[channelIndex];
    lastProcessedUartError = BATTERY_MODBUS_ERROR_FLAG |
                             reason |
                             ((uint32_t)modbusRetryCount << 8) |
                             (detail & 0xFFu);
}

static void BatteryModbus_ClearPendingFrame(UartContext_t *context)
{
    if (context == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    context->dataReady = false;
    context->rxLength = 0u;
    taskEXIT_CRITICAL();
}

static void BatteryModbus_FinishChannel(uint8_t channelIndex, uint32_t currentTick)
{
    uint32_t elapsed;

    modbusAwaitingResponse = false;
    modbusRetryCount = 0u;
    modbusNextChannel = BatteryModbus_NextChannel(channelIndex);

    elapsed = currentTick - modbusRequestTick;
    if (elapsed < BATTERY_MODBUS_POLL_SLOT_MS)
    {
        modbusNextPollTick = modbusRequestTick + BATTERY_MODBUS_POLL_SLOT_MS;
    }
    else
    {
        modbusNextPollTick = currentTick;
    }
}

/* Забирает принятый UART-кадр из контекста в локальный буфер задачи. */
static size_t BatteryModbus_TakeReceivedFrame(UartContext_t *context, uint8_t *localBuffer, size_t localBufferSize)
{
    size_t length = 0u;

    if ((context == NULL) || (localBuffer == NULL) || (localBufferSize == 0u))
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    length = context->rxLength;
    if (length > localBufferSize)
    {
        length = localBufferSize;
    }
    memcpy(localBuffer, context->rxBuffer, length);
    if (length < localBufferSize)
    {
        memset(&localBuffer[length], 0, localBufferSize - length);
    }
    context->dataReady = false;
    context->rxLength = 0u;
    taskEXIT_CRITICAL();

    return length;
}

static bool BatteryModbus_SendReadRequest(uint8_t channelIndex,
                                          UartContext_t *context,
                                          ModbusChannelBuffer_t *modbusBuffer,
                                          uint32_t currentTick)
{
    HAL_StatusTypeDef status;
    osStatus mutexStatus;

    if ((channelIndex >= UART_CHANNEL_COUNT) || (context == NULL) || (modbusBuffer == NULL))
    {
        return false;
    }

    if (context->mutex == NULL)
    {
        BatteryModbus_LogError(channelIndex, BATTERY_MODBUS_ERROR_MUTEX, 0u);
        return false;
    }

    mutexStatus = osMutexWait(context->mutex, 0u);
    if (mutexStatus != osOK)
    {
        BatteryModbus_LogError(channelIndex, BATTERY_MODBUS_ERROR_MUTEX, (uint32_t)mutexStatus);
        return false;
    }

    if (!Modbus_PrepareReadRequest(modbusBuffer,
                                   modbusStartRegister,
                                   MODBUS_REGISTER_COUNT,
                                   MODBUS_DEFAULT_SLAVE_ADDRESS))
    {
        osMutexRelease(context->mutex);
        BatteryModbus_LogError(channelIndex, BATTERY_MODBUS_ERROR_TX, 0u);
        return false;
    }

    BatteryModbus_ClearPendingFrame(context);

    status = HAL_UART_Transmit(context->handle,
                               modbusBuffer->request,
                               (uint16_t)modbusBuffer->requestLength,
                               BATTERY_MODBUS_TX_TIMEOUT_MS);
    osMutexRelease(context->mutex);

    if (status != HAL_OK)
    {
        BatteryModbus_LogError(channelIndex,
                               BATTERY_MODBUS_ERROR_TX,
                               context->handle->ErrorCode | ((uint32_t)status << 4));
        context->reinit_needed = true;
        return false;
    }

    modbusActiveChannel = channelIndex;
    modbusRequestTick = currentTick;
    modbusAwaitingResponse = true;
    return true;
}

static void BatteryModbus_StartOrRetryChannel(uint8_t channelIndex, uint32_t currentTick)
{
    UartContext_t *context;
    ModbusChannelBuffer_t *modbusBuffer;

    if (channelIndex >= UART_CHANNEL_COUNT)
    {
        return;
    }

    if (Slave_IsUpdateActive(channelIndex))
    {
        modbusRequestTick = currentTick;
        BatteryModbus_FinishChannel(channelIndex, currentTick);
        return;
    }

    context = &uartContexts[channelIndex];
    modbusBuffer = &modbusBuffers[channelIndex];

    modbusActiveChannel = channelIndex;
    modbusRequestTick = currentTick;
    modbusAwaitingResponse = true;

    if (!BatteryModbus_SendReadRequest(channelIndex, context, modbusBuffer, currentTick))
    {
        return;
    }
}

static void BatteryModbus_FailAttempt(uint8_t channelIndex,
                                      uint32_t reason,
                                      uint32_t detail,
                                      uint32_t currentTick)
{
    if (channelIndex >= UART_CHANNEL_COUNT)
    {
        return;
    }

    BatteryModbus_LogError(channelIndex, reason, detail);

    if (modbusRetryCount < BATTERY_MODBUS_MAX_RETRIES)
    {
        modbusRetryCount++;
        BatteryModbus_StartOrRetryChannel(channelIndex, currentTick);
        return;
    }

    BatteryModbus_LogError(channelIndex, BATTERY_MODBUS_ERROR_RETRY_LIMIT, reason);
    modbusSlaveData[channelIndex].valid = false;
    BAT_SetIndicator(channelIndex);
    BatteryModbus_FinishChannel(channelIndex, currentTick);
}

static void BatteryModbus_HandleHardwareError(uint8_t channelIndex,
                                              UartContext_t *context,
                                              uint32_t currentTick)
{
    uint32_t errorCode;

    if ((channelIndex >= UART_CHANNEL_COUNT) || (context == NULL) || !context->errorDetected)
    {
        return;
    }

    errorCode = context->errorCode;
    context->errorDetected = false;
    context->errorCode = 0u;

    BatteryModbus_LogError(channelIndex, BATTERY_MODBUS_ERROR_UART, errorCode);

    if (modbusAwaitingResponse && (channelIndex == modbusActiveChannel))
    {
        BatteryModbus_FailAttempt(channelIndex,
                                  BATTERY_MODBUS_ERROR_UART,
                                  errorCode,
                                  currentTick);
    }
}

/* Обрабатывает одиночный байт ACK/NACK от ROM bootloader ведомого контроллера. */
static void BatteryModbus_HandleBootloaderByte(uint8_t channelIndex, uint8_t value)
{
    if (channelIndex >= UART_CHANNEL_COUNT)
    {
        return;
    }

    if ((value == BOOTLOADER_ACK) || (value == BOOTLOADER_NACK))
    {
        slave.device[channelIndex].ack = value;
    }
    else
    {
        modbusSlaveData[channelIndex].valid = false;
    }
}

/* Разбирает полный UART-кадр как bootloader-ответ или Modbus-ответ батареи. */
static BatteryModbusFrameResult_t BatteryModbus_HandleFrame(uint8_t channelIndex,
                                                            ModbusChannelBuffer_t *modbusBuffer,
                                                            const uint8_t *localBuffer,
                                                            size_t length)
{
    if ((channelIndex >= UART_CHANNEL_COUNT) || (modbusBuffer == NULL) ||
        (localBuffer == NULL) || (length == 0u))
    {
        return BATTERY_MODBUS_FRAME_NONE;
    }

    if (length == 1u)
    {
        BatteryModbus_HandleBootloaderByte(channelIndex, localBuffer[0]);
        return BATTERY_MODBUS_FRAME_BOOTLOADER;
    }

    (void)Modbus_SaveResponse(modbusBuffer, localBuffer, length);

    if (!ParseModbusResponse(localBuffer, length, &modbusSlaveData[channelIndex]))
    {
        return BATTERY_MODBUS_FRAME_INVALID;
    }

    lastProcessedUartNumber = uartIndexToNumber[channelIndex];
    BAT_UpdateAllIndicators();
    return BATTERY_MODBUS_FRAME_VALID;
}

/* Отправляет периодический Modbus-запрос чтения регистров в выбранный канал. */
static void BatteryModbus_ProcessFrameResult(uint8_t channelIndex,
                                             BatteryModbusFrameResult_t frameResult,
                                             uint32_t currentTick)
{
    if ((channelIndex >= UART_CHANNEL_COUNT) || !modbusAwaitingResponse ||
        (channelIndex != modbusActiveChannel))
    {
        return;
    }

    if (frameResult == BATTERY_MODBUS_FRAME_VALID)
    {
        BatteryModbus_FinishChannel(channelIndex, currentTick);
        return;
    }

    if (frameResult == BATTERY_MODBUS_FRAME_INVALID)
    {
        BatteryModbus_FailAttempt(channelIndex,
                                  BATTERY_MODBUS_ERROR_RESPONSE,
                                  0u,
                                  currentTick);
    }
}

static void BatteryModbus_PollScheduler(uint32_t currentTick)
{
    if (modbusAwaitingResponse)
    {
        if ((currentTick - modbusRequestTick) >= BATTERY_MODBUS_RESPONSE_TIMEOUT_MS)
        {
            BatteryModbus_FailAttempt(modbusActiveChannel,
                                      BATTERY_MODBUS_ERROR_TIMEOUT,
                                      0u,
                                      currentTick);
        }
        return;
    }

    if (!BatteryModbus_TimeReached(currentTick, modbusNextPollTick))
    {
        return;
    }

    BatteryModbus_StartOrRetryChannel(modbusNextChannel, currentTick);
}

/* Основная задача опроса батарейных блоков по UART/Modbus. */
void BatteryModbus_Task(void const *argument)
{
    uint8_t localBuffer[UART_RX_CHUNK_SIZE] = {0};

    (void)argument;

    for (;;)
    {
        uint32_t currentTick = HAL_GetTick();

#if UARTS_ENABLE == 1
        for (uint8_t index = 0u; index < UART_CHANNEL_COUNT; ++index)
        {
            UartContext_t *context = &uartContexts[index];
            ModbusChannelBuffer_t *modbusBuffer = &modbusBuffers[index];

            Uart_CheckAndRecover(context);
            BatteryModbus_HandleHardwareError(index, context, currentTick);

            if (context->dataReady)
            {
                BatteryModbusFrameResult_t frameResult;
                size_t length = BatteryModbus_TakeReceivedFrame(context,
                                                                localBuffer,
                                                                sizeof(localBuffer));
                frameResult = BatteryModbus_HandleFrame(index, modbusBuffer, localBuffer, length);
                BatteryModbus_ProcessFrameResult(index, frameResult, currentTick);
            }
        }

        BatteryModbus_PollScheduler(currentTick);
#endif

        osDelay(1);
    }
}
