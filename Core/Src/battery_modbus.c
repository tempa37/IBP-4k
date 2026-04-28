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
static uint32_t modbusLastPollTick[UART_CHANNEL_COUNT] = {0u};

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
static void BatteryModbus_HandleFrame(uint8_t channelIndex,
                                      ModbusChannelBuffer_t *modbusBuffer,
                                      const uint8_t *localBuffer,
                                      size_t length)
{
    if ((channelIndex >= UART_CHANNEL_COUNT) || (modbusBuffer == NULL) ||
        (localBuffer == NULL) || (length == 0u))
    {
        return;
    }

    if (length == 1u)
    {
        BatteryModbus_HandleBootloaderByte(channelIndex, localBuffer[0]);
        return;
    }

    Slave_OnModbusFrameReceived(channelIndex, localBuffer, length);
    (void)Modbus_SaveResponse(modbusBuffer, localBuffer, length);

    if (!ParseModbusResponse(localBuffer, length, &modbusSlaveData[channelIndex]))
    {
        modbusSlaveData[channelIndex].valid = false;
    }

    lastProcessedUartNumber = uartIndexToNumber[channelIndex];
    BAT_UpdateAllIndicators();
}

/* Отправляет периодический Modbus-запрос чтения регистров в выбранный канал. */
static void BatteryModbus_PollChannel(uint8_t channelIndex,
                                      UartContext_t *context,
                                      ModbusChannelBuffer_t *modbusBuffer,
                                      uint32_t currentTick)
{
    static const uint32_t modbusPollIntervalMs = 500u;

    if ((channelIndex >= UART_CHANNEL_COUNT) || (context == NULL) || (modbusBuffer == NULL))
    {
        return;
    }

    if (Slave_IsUpdateActive(channelIndex) ||
        ((currentTick - modbusLastPollTick[channelIndex]) < modbusPollIntervalMs))
    {
        return;
    }

    if ((context->mutex != NULL) && (osMutexWait(context->mutex, osWaitForever) == osOK))
    {
        if (Modbus_PrepareReadRequest(modbusBuffer,
                                      modbusStartRegister,
                                      MODBUS_REGISTER_COUNT,
                                      MODBUS_DEFAULT_SLAVE_ADDRESS))
        {
            HAL_StatusTypeDef status = HAL_UART_Transmit(context->handle,
                                                         modbusBuffer->request,
                                                         (uint16_t)modbusBuffer->requestLength,
                                                         1000u);
            if (status != HAL_OK)
            {
                lastProcessedUartNumber = uartIndexToNumber[channelIndex];
                lastProcessedUartError = context->handle->ErrorCode | ((uint32_t)status << 24);
            }
        }

        osMutexRelease(context->mutex);
    }

    modbusLastPollTick[channelIndex] = currentTick;
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

            if (context->dataReady)
            {
                size_t length = BatteryModbus_TakeReceivedFrame(context,
                                                                localBuffer,
                                                                sizeof(localBuffer));
                BatteryModbus_HandleFrame(index, modbusBuffer, localBuffer, length);
            }

            BatteryModbus_PollChannel(index, context, modbusBuffer, currentTick);
        }
#endif

        osDelay(10);
    }
}
