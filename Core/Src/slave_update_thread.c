#include "slave_update_thread.h"

#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "cmsis_os.h"
#include "modbuc.h"

#include <string.h>

#define SLAVE_UPDATE_INVALID_INDEX  0xFFu

extern ModbusSlaveData_t modbusSlaveData[UART_CHANNEL_COUNT];

typedef enum
{
    SLAVE_UPD_IDLE = 0,
    SLAVE_UPD_SEND_ENTER_BOOTLOADER,
    SLAVE_UPD_WAIT_BOOTLOADER_ENTRY,
    SLAVE_UPD_SEND_SYNC,
    SLAVE_UPD_WAIT_ACK_SYNC,
    SLAVE_UPD_SEND_EXT_ERASE_CMD,
    SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD,
    SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD,
    SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD,
    SLAVE_UPD_PREPARE_BLOCK,
    SLAVE_UPD_SEND_WRITE_CMD,
    SLAVE_UPD_WAIT_ACK_WRITE_CMD,
    SLAVE_UPD_SEND_ADDRESS,
    SLAVE_UPD_WAIT_ACK_ADDRESS,
    SLAVE_UPD_SEND_DATA,
    SLAVE_UPD_WAIT_ACK_DATA,
    SLAVE_UPD_NEXT_BLOCK,
    SLAVE_UPD_SEND_GO_CMD,
    SLAVE_UPD_WAIT_ACK_GO_CMD,
    SLAVE_UPD_SEND_GO_ADDRESS,
    SLAVE_UPD_WAIT_ACK_GO_ADDRESS,
    SLAVE_UPD_DONE,
    SLAVE_UPD_ERROR
} SlaveUpdatePhase_t;

typedef struct
{
    SlaveUpdatePhase_t phase;
    uint8_t slave_number;
    uint32_t source_address;
    uint32_t app_start_address;
    uint32_t target_address;
    uint32_t bytes_written;
    uint32_t total_bytes;
    uint8_t retry_count;
    uint32_t last_command_tick;
    uint16_t payload_size;
    uint16_t padded_size;
    uint8_t write_buffer[SLAVE_FW_BLOCK_SIZE];
} SlaveUpdateContext_t;

static SlaveUpdateContext_t slave_update_ctx[UART_CHANNEL_COUNT] = {0};
static volatile uint8_t slave_update_pending_mask = 0u;
static volatile uint8_t slave_update_success_mask = 0u;
static volatile uint8_t slave_update_error_mask = 0u;
static volatile uint8_t slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
static volatile uint8_t slave_update_auto_pending = 0u;
volatile SlaveUpdateDebugInfo_t g_slave_update_debug = {0};

static bool Slave_SendBootloaderCommand(uint8_t slave_num, const uint8_t *data, uint16_t length)
{
    UartContext_t *ctx;

    if ((slave_num >= UART_CHANNEL_COUNT) || (data == NULL) || (length == 0u))
    {
        return false;
    }

    ctx = &uartContexts[slave_num];
    if ((ctx == NULL) || (ctx->handle == NULL) || (ctx->mutex == NULL))
    {
        return false;
    }

    if (osMutexWait(ctx->mutex, osWaitForever) != osOK)
    {
        return false;
    }

    if (HAL_UART_Transmit(ctx->handle, (uint8_t *)data, length, 1000u) != HAL_OK)
    {
        osMutexRelease(ctx->mutex);
        return false;
    }

    osMutexRelease(ctx->mutex);
    return true;
}

static bool Slave_TakeBootloaderResponse(uint8_t slave_num, uint8_t *response)
{
    uint8_t value;

    if ((slave_num >= UART_CHANNEL_COUNT) || (response == NULL))
    {
        return false;
    }

    value = slave.device[slave_num].ack;
    if ((value != BOOTLOADER_ACK) && (value != BOOTLOADER_NACK))
    {
        return false;
    }

    slave.device[slave_num].ack = 0u;
    *response = value;
    return true;
}

static uint8_t Slave_GetChannelBit(uint8_t slave_num)
{
    if (slave_num >= UART_CHANNEL_COUNT)
    {
        return 0u;
    }

    return (uint8_t)(1u << slave_num);
}

static uint8_t Slave_GetConnectedMask(void)
{
    uint8_t mask = 0u;

    for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
    {
        if (modbusSlaveData[slave_num].valid)
        {
            mask |= Slave_GetChannelBit(slave_num);
        }
    }

    return mask;
}

static bool Slave_HasStoredImageReady(void)
{
    return (slave.os_in_flash != 0u) &&
           (slave.os_size_bytes > 0u) &&
           (slave.os_size_bytes <= FLASH_FW_STORAGE_SIZE) &&
           (slave.os_size_bytes <= FLASH_APP_SIZE);
}

static void Slave_ResetBatchState(void)
{
    slave_update_pending_mask = 0u;
    slave_update_success_mask = 0u;
    slave_update_error_mask = 0u;
    slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
}

static void Slave_UpdateDebugSnapshot(uint32_t current_tick)
{
    const SlaveUpdateContext_t *ctx = NULL;
    uint8_t live_slave_num = slave_update_active_slave;

    if (live_slave_num < UART_CHANNEL_COUNT)
    {
        ctx = &slave_update_ctx[live_slave_num];
    }
    else
    {
        for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
        {
            if (slave_update_ctx[slave_num].phase != SLAVE_UPD_IDLE)
            {
                live_slave_num = slave_num;
                ctx = &slave_update_ctx[slave_num];
                break;
            }
        }
    }

    g_slave_update_debug.last_tick = current_tick;
    g_slave_update_debug.pending_mask = slave_update_pending_mask;
    g_slave_update_debug.success_mask = slave_update_success_mask;
    g_slave_update_debug.error_mask = slave_update_error_mask;
    g_slave_update_debug.active_slave = slave_update_active_slave;
    g_slave_update_debug.auto_pending = slave_update_auto_pending;
    g_slave_update_debug.live_slave_num = (ctx != NULL) ? live_slave_num : SLAVE_UPDATE_INVALID_INDEX;
    g_slave_update_debug.live_phase = (ctx != NULL) ? (uint8_t)ctx->phase : (uint8_t)SLAVE_UPD_IDLE;
    g_slave_update_debug.live_retry_count = (ctx != NULL) ? ctx->retry_count : 0u;
    g_slave_update_debug.live_ack = ((ctx != NULL) && (live_slave_num < UART_CHANNEL_COUNT))
                                  ? slave.device[live_slave_num].ack
                                  : 0u;
    g_slave_update_debug.live_last_command_tick = (ctx != NULL) ? ctx->last_command_tick : 0u;
    g_slave_update_debug.live_bytes_written = (ctx != NULL) ? ctx->bytes_written : 0u;
    g_slave_update_debug.live_total_bytes = (ctx != NULL) ? ctx->total_bytes : 0u;
    g_slave_update_debug.live_target_address = (ctx != NULL) ? ctx->target_address : 0u;
}

static void Slave_RecordDebugEvent(uint8_t slave_num,
                                   uint8_t event_code,
                                   SlaveUpdatePhase_t phase,
                                   uint32_t current_tick)
{
    g_slave_update_debug.event_counter++;
    g_slave_update_debug.last_event = event_code;
    g_slave_update_debug.last_slave_num = slave_num;
    g_slave_update_debug.last_phase = (uint8_t)phase;
    Slave_UpdateDebugSnapshot(current_tick);
}

static void Slave_FinishActiveUpdate(uint8_t slave_num, bool success)
{
    uint8_t channel_bit = Slave_GetChannelBit(slave_num);

    if (channel_bit == 0u)
    {
        return;
    }

    if (success)
    {
        slave_update_success_mask |= channel_bit;
    }
    else
    {
        slave_update_error_mask |= channel_bit;
    }

    if (slave_update_active_slave == slave_num)
    {
        slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
    }
}

static void Slave_MarkMaskedChannelsAsError(uint8_t mask)
{
    slave_update_error_mask |= mask;
}

static void Slave_StartNextPendingUpdate(void)
{
    if (!Slave_HasStoredImageReady())
    {
        Slave_MarkMaskedChannelsAsError(slave_update_pending_mask);
        slave_update_pending_mask = 0u;
        slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
        Slave_UpdateDebugSnapshot(HAL_GetTick());
        return;
    }

    if (slave_update_active_slave != SLAVE_UPDATE_INVALID_INDEX)
    {
        return;
    }

    while (slave_update_pending_mask != 0u)
    {
        for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
        {
            uint8_t channel_bit = Slave_GetChannelBit(slave_num);

            if ((slave_update_pending_mask & channel_bit) == 0u)
            {
                continue;
            }

            slave_update_pending_mask &= (uint8_t)(~channel_bit);

            if (!modbusSlaveData[slave_num].valid)
            {
                slave_update_error_mask |= channel_bit;
                continue;
            }

            slave_update_active_slave = slave_num;
            slave.device[slave_num].need_update = 1u;
            Slave_UpdateDebugSnapshot(HAL_GetTick());
            return;
        }
    }

    slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
    Slave_UpdateDebugSnapshot(HAL_GetTick());
}

static bool Slave_RequestUpdateMask(uint8_t request_mask)
{
    if ((SLAVE_FW_SEND_ENABLE == 0u) || !Slave_HasStoredImageReady())
    {
        return false;
    }

    request_mask &= Slave_GetConnectedMask();
    if (request_mask == 0u)
    {
        return false;
    }

    if (Slave_IsAnyUpdateRunning())
    {
        return false;
    }

    taskENTER_CRITICAL();
    Slave_ResetBatchState();
    slave_update_pending_mask = request_mask;
    slave_update_auto_pending = 0u;
    Slave_UpdateDebugSnapshot(HAL_GetTick());
    taskEXIT_CRITICAL();

    return true;
}

static bool Slave_IsAckWaitPhase(SlaveUpdatePhase_t phase)
{
    switch (phase)
    {
        case SLAVE_UPD_WAIT_ACK_SYNC:
        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD:
        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD:
        case SLAVE_UPD_WAIT_ACK_WRITE_CMD:
        case SLAVE_UPD_WAIT_ACK_ADDRESS:
        case SLAVE_UPD_WAIT_ACK_DATA:
        case SLAVE_UPD_WAIT_ACK_GO_CMD:
        case SLAVE_UPD_WAIT_ACK_GO_ADDRESS:
            return true;

        default:
            return false;
    }
}

static uint32_t Slave_GetPhaseTimeoutMs(SlaveUpdatePhase_t phase)
{
    if (phase == SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD)
    {
        return SLAVE_UPDATE_ERASE_TIMEOUT_MS;
    }

    return SLAVE_UPDATE_TIMEOUT_MS;
}

static SlaveUpdatePhase_t Slave_GetRetryPhase(SlaveUpdatePhase_t phase)
{
    switch (phase)
    {
        case SLAVE_UPD_WAIT_ACK_SYNC:
            return SLAVE_UPD_SEND_ENTER_BOOTLOADER;

        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD:
            return SLAVE_UPD_SEND_EXT_ERASE_CMD;

        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD:
            return SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD;

        case SLAVE_UPD_WAIT_ACK_WRITE_CMD:
            return SLAVE_UPD_SEND_WRITE_CMD;

        case SLAVE_UPD_WAIT_ACK_ADDRESS:
            return SLAVE_UPD_SEND_ADDRESS;

        case SLAVE_UPD_WAIT_ACK_DATA:
            return SLAVE_UPD_SEND_DATA;

        case SLAVE_UPD_WAIT_ACK_GO_CMD:
            return SLAVE_UPD_SEND_GO_CMD;

        case SLAVE_UPD_WAIT_ACK_GO_ADDRESS:
            return SLAVE_UPD_SEND_GO_ADDRESS;

        default:
            return SLAVE_UPD_ERROR;
    }
}

static void Slave_ScheduleRetryOrError(SlaveUpdateContext_t *ctx, uint32_t current_tick)
{
    if (ctx == NULL)
    {
        return;
    }

    if (ctx->retry_count < SLAVE_UPDATE_RETRY_MAX)
    {
        ctx->retry_count++;
        ctx->phase = Slave_GetRetryPhase(ctx->phase);
        ctx->last_command_tick = current_tick;
        slave.device[ctx->slave_number].ack = 0u;
    }
    else
    {
        ctx->phase = SLAVE_UPD_ERROR;
    }
}

static bool Slave_PrepareNextDataBlock(SlaveUpdateContext_t *ctx)
{
    uint32_t bytes_remaining;
    const uint8_t *source_ptr;

    if (ctx == NULL)
    {
        return false;
    }

    if (ctx->bytes_written >= ctx->total_bytes)
    {
        ctx->phase = SLAVE_UPD_SEND_GO_CMD;
        return true;
    }

    bytes_remaining = ctx->total_bytes - ctx->bytes_written;
    ctx->payload_size = (uint16_t)((bytes_remaining > SLAVE_FW_BLOCK_SIZE) ? SLAVE_FW_BLOCK_SIZE : bytes_remaining);
    ctx->padded_size = (uint16_t)((ctx->payload_size + 3u) & ~0x03u);

    memset(ctx->write_buffer, 0xFF, sizeof(ctx->write_buffer));
    source_ptr = (const uint8_t *)(ctx->source_address + ctx->bytes_written);
    memcpy(ctx->write_buffer, source_ptr, ctx->payload_size);

    ctx->phase = SLAVE_UPD_SEND_WRITE_CMD;
    return true;
}

void Slave_CancelPendingUpdates(void)
{
    taskENTER_CRITICAL();

    for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
    {
        slave.device[slave_num].need_update = 0u;
        slave.device[slave_num].ack = 0u;
        memset(&slave_update_ctx[slave_num], 0, sizeof(slave_update_ctx[slave_num]));
    }

    Slave_ResetBatchState();
    slave_update_auto_pending = 0u;
    Slave_UpdateDebugSnapshot(HAL_GetTick());
    taskEXIT_CRITICAL();
}

bool Slave_IsAnyUpdateRunning(void)
{
    if ((slave_update_pending_mask != 0u) ||
        (slave_update_active_slave != SLAVE_UPDATE_INVALID_INDEX))
    {
        return true;
    }

    for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
    {
        if ((slave.device[slave_num].need_update != 0u) ||
            (slave_update_ctx[slave_num].phase != SLAVE_UPD_IDLE))
        {
            return true;
        }
    }

    return false;
}

bool Slave_RequestSingleUpdate(uint8_t slave_num)
{
    uint8_t request_mask = Slave_GetChannelBit(slave_num);
    return Slave_RequestUpdateMask(request_mask);
}

bool Slave_RequestConnectedUpdate(void)
{
    return Slave_RequestUpdateMask(0xFFu);
}

bool Slave_RequestAutoConnectedUpdate(void)
{
    if ((SLAVE_FW_SEND_ENABLE == 0u) || !Slave_HasStoredImageReady())
    {
        return false;
    }

    taskENTER_CRITICAL();
    slave_update_auto_pending = 1u;
    Slave_UpdateDebugSnapshot(HAL_GetTick());
    taskEXIT_CRITICAL();

    return true;
}

bool Slave_IsUpdateActive(uint8_t slave_num)
{
    if (slave_num >= UART_CHANNEL_COUNT)
    {
        return false;
    }

    return (slave.device[slave_num].need_update != 0u) ||
           (slave_update_ctx[slave_num].phase != SLAVE_UPD_IDLE);
}

void Slave_UpdateProcess(void)
{
    uint32_t current_tick = HAL_GetTick();

    if (SLAVE_FW_SEND_ENABLE == 0u)
    {
        return;
    }

    if ((slave_update_auto_pending != 0u) &&
        !Slave_IsAnyUpdateRunning())
    {
        uint8_t connected_mask = Slave_GetConnectedMask();

        if (connected_mask != 0u)
        {
            taskENTER_CRITICAL();
            Slave_ResetBatchState();
            slave_update_pending_mask = connected_mask;
            slave_update_auto_pending = 0u;
            Slave_UpdateDebugSnapshot(current_tick);
            taskEXIT_CRITICAL();
        }
    }

    if (slave_update_active_slave == SLAVE_UPDATE_INVALID_INDEX)
    {
        Slave_StartNextPendingUpdate();
    }

    for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
    {
        SlaveUpdateContext_t *ctx = &slave_update_ctx[slave_num];
        uint8_t response = 0u;

        if ((ctx->phase == SLAVE_UPD_IDLE) &&
            (slave_update_active_slave == slave_num) &&
            (slave.device[slave_num].need_update != 0u) &&
            (slave.os_in_flash != 0u) &&
            (slave.os_size_bytes > 0u))
        {
            memset(ctx, 0, sizeof(*ctx));
            ctx->slave_number = slave_num;
            ctx->source_address = FLASH_FW_STORAGE_START_ADDR;
            ctx->total_bytes = slave.os_size_bytes;

            if ((ctx->total_bytes == 0u) ||
                (ctx->total_bytes > FLASH_FW_STORAGE_SIZE) ||
                (ctx->total_bytes > FLASH_APP_SIZE))
            {
                ctx->phase = SLAVE_UPD_ERROR;
            }
            else
            {
                ctx->app_start_address = SLAVE_FLASH_TARGET_START_ADDR;
                ctx->target_address = ctx->app_start_address;
                ctx->phase = SLAVE_UPD_SEND_ENTER_BOOTLOADER;
            }
        }

        if (Slave_IsAckWaitPhase(ctx->phase) &&
            ((current_tick - ctx->last_command_tick) > Slave_GetPhaseTimeoutMs(ctx->phase)))
        {
            Slave_ScheduleRetryOrError(ctx, current_tick);
        }

        switch (ctx->phase)
        {
            case SLAVE_UPD_SEND_ENTER_BOOTLOADER:
            {
                uint8_t modbus_cmd[8];
                uint16_t crc;

                modbus_cmd[0] = 0x01u;
                modbus_cmd[1] = 0x06u;
                modbus_cmd[2] = 0x00u;
                modbus_cmd[3] = 0x00u;
                modbus_cmd[4] = 0x12u;
                modbus_cmd[5] = 0x34u;
                crc = Modbus_CalculateCRC(modbus_cmd, 6u);
                modbus_cmd[6] = (uint8_t)(crc & 0xFFu);
                modbus_cmd[7] = (uint8_t)((crc >> 8) & 0xFFu);

                if (Slave_SendBootloaderCommand(slave_num, modbus_cmd, sizeof(modbus_cmd)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_BOOTLOADER_ENTRY;
                    ctx->last_command_tick = current_tick;
                    ctx->retry_count = 0u;
                    slave.device[slave_num].ack = 0u;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_BOOTLOADER_ENTRY:
                if ((current_tick - ctx->last_command_tick) >= SLAVE_BOOTLOADER_ENTRY_DELAY_MS)
                {
                    ctx->phase = SLAVE_UPD_SEND_SYNC;
                }
                break;

            case SLAVE_UPD_SEND_SYNC:
            {
                const uint8_t sync_cmd = BOOTLOADER_SYNC;

                if (Slave_SendBootloaderCommand(slave_num, &sync_cmd, 1u))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_SYNC;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_SYNC:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_EXT_ERASE_CMD;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_EXT_ERASE_CMD:
            {
                const uint8_t erase_cmd[2] = {BOOTLOADER_EXT_ERASE, (uint8_t)(BOOTLOADER_EXT_ERASE ^ 0xFFu)};

                if (Slave_SendBootloaderCommand(slave_num, erase_cmd, sizeof(erase_cmd)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD:
            {
                const uint8_t mass_erase_payload[3] = {0xFFu, 0xFFu, 0x00u};

                if (Slave_SendBootloaderCommand(slave_num, mass_erase_payload, sizeof(mass_erase_payload)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_PREPARE_BLOCK;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_PREPARE_BLOCK:
                if (!Slave_PrepareNextDataBlock(ctx))
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;

            case SLAVE_UPD_SEND_WRITE_CMD:
            {
                const uint8_t write_cmd[2] = {BOOTLOADER_WRITE, (uint8_t)(BOOTLOADER_WRITE ^ 0xFFu)};

                if (Slave_SendBootloaderCommand(slave_num, write_cmd, sizeof(write_cmd)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_WRITE_CMD;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_WRITE_CMD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_ADDRESS;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_ADDRESS:
            {
                uint8_t addr_cmd[5];

                addr_cmd[0] = (uint8_t)((ctx->target_address >> 24) & 0xFFu);
                addr_cmd[1] = (uint8_t)((ctx->target_address >> 16) & 0xFFu);
                addr_cmd[2] = (uint8_t)((ctx->target_address >> 8) & 0xFFu);
                addr_cmd[3] = (uint8_t)(ctx->target_address & 0xFFu);
                addr_cmd[4] = (uint8_t)(addr_cmd[0] ^ addr_cmd[1] ^ addr_cmd[2] ^ addr_cmd[3]);

                if (Slave_SendBootloaderCommand(slave_num, addr_cmd, sizeof(addr_cmd)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_ADDRESS;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_ADDRESS:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_DATA;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_DATA:
            {
                uint8_t data_packet[SLAVE_FW_BLOCK_SIZE + 2u];
                uint8_t checksum = 0u;

                data_packet[0] = (uint8_t)(ctx->padded_size - 1u);
                checksum ^= data_packet[0];

                for (uint16_t index = 0u; index < ctx->padded_size; ++index)
                {
                    data_packet[1u + index] = ctx->write_buffer[index];
                    checksum ^= ctx->write_buffer[index];
                }

                data_packet[1u + ctx->padded_size] = checksum;

                if (Slave_SendBootloaderCommand(slave_num,
                                                data_packet,
                                                (uint16_t)(ctx->padded_size + 2u)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_DATA;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_DATA:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->bytes_written += ctx->payload_size;
                        ctx->target_address += ctx->padded_size;
                        ctx->phase = SLAVE_UPD_NEXT_BLOCK;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_NEXT_BLOCK:
                ctx->phase = (ctx->bytes_written < ctx->total_bytes)
                           ? SLAVE_UPD_PREPARE_BLOCK
                           : SLAVE_UPD_SEND_GO_CMD;
                break;

            case SLAVE_UPD_SEND_GO_CMD:
            {
                const uint8_t go_cmd[2] = {BOOTLOADER_GO, (uint8_t)(BOOTLOADER_GO ^ 0xFFu)};

                if (Slave_SendBootloaderCommand(slave_num, go_cmd, sizeof(go_cmd)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_GO_CMD;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_GO_CMD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_GO_ADDRESS;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_GO_ADDRESS:
            {
                uint8_t addr_cmd[5];

                addr_cmd[0] = (uint8_t)((ctx->app_start_address >> 24) & 0xFFu);
                addr_cmd[1] = (uint8_t)((ctx->app_start_address >> 16) & 0xFFu);
                addr_cmd[2] = (uint8_t)((ctx->app_start_address >> 8) & 0xFFu);
                addr_cmd[3] = (uint8_t)(ctx->app_start_address & 0xFFu);
                addr_cmd[4] = (uint8_t)(addr_cmd[0] ^ addr_cmd[1] ^ addr_cmd[2] ^ addr_cmd[3]);

                if (Slave_SendBootloaderCommand(slave_num, addr_cmd, sizeof(addr_cmd)))
                {
                    ctx->phase = SLAVE_UPD_WAIT_ACK_GO_ADDRESS;
                    ctx->last_command_tick = current_tick;
                }
                else
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_GO_ADDRESS:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_DONE;
                        ctx->retry_count = 0u;
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx, current_tick);
                    }
                }
                break;

            case SLAVE_UPD_DONE:
                Slave_FinishActiveUpdate(slave_num, true);
                Slave_RecordDebugEvent(slave_num,
                                       SLAVE_UPDATE_DEBUG_EVENT_DONE,
                                       ctx->phase,
                                       current_tick);
                slave.device[slave_num].need_update = 0u;
                slave.device[slave_num].ack = 0u;
                memset(ctx, 0, sizeof(*ctx));
                break;

            case SLAVE_UPD_ERROR:
                Slave_FinishActiveUpdate(slave_num, false);
                Slave_RecordDebugEvent(slave_num,
                                       SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                                       ctx->phase,
                                       current_tick);
                slave.device[slave_num].need_update = 0u;
                slave.device[slave_num].ack = 0u;
                memset(ctx, 0, sizeof(*ctx));
                break;

            case SLAVE_UPD_IDLE:
            default:
                break;
        }
    }

    Slave_UpdateDebugSnapshot(current_tick);
}
