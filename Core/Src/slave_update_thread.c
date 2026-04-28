#include "slave_update_thread.h"

#include "battery_data.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "cmsis_os.h"
#include "modbuc.h"

#include <string.h>

#define SLAVE_UPDATE_INVALID_INDEX  0xFFu
#define SLAVE_BOOT_RESP_NONE        0u
#define SLAVE_BOOT_RESP_OK          1u
#define SLAVE_BOOT_RESP_INVALID     2u
#define SLAVE_EXT_ERASE_PAGES_PER_CMD    16u
#define SLAVE_EXT_ERASE_PAYLOAD_MAX_SIZE (2u + (2u * SLAVE_EXT_ERASE_PAGES_PER_CMD) + 1u)

SlaveSystem_t slave = {0};

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
    uint8_t boot_resp_received;
    uint16_t erase_next_page;
    uint16_t erase_last_page;
    uint16_t erase_pages_in_block;
    uint16_t payload_size;
    uint16_t padded_size;
    uint8_t write_buffer[SLAVE_FW_BLOCK_SIZE];
} SlaveUpdateContext_t;

static SlaveUpdateContext_t slave_update_ctx[UART_CHANNEL_COUNT];
static volatile uint8_t slave_update_pending_mask = 0u;
static volatile uint8_t slave_update_success_mask = 0u;
static volatile uint8_t slave_update_error_mask = 0u;
static volatile uint8_t slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
static volatile uint8_t slave_update_auto_pending = 0u;
static volatile uint8_t slave_boot_resp_state[UART_CHANNEL_COUNT] = {0};
volatile SlaveUpdateDebugInfo_t g_slave_update_debug = {0};
volatile SlaveUpdateTraceEntry_t g_slave_update_trace[SLAVE_UPDATE_TRACE_DEPTH] = {0};

/* Отправляет команду во встроенный bootloader выбранного ведомого контроллера по UART. */
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

/* Забирает один ACK/NACK байт bootloader из RAM-состояния ведомого канала. */
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

/* Анализирует Modbus-кадр на предмет ответа на команду входа ведомого контроллера в bootloader. */
void Slave_OnModbusFrameReceived(uint8_t slave_num, const uint8_t *frame, size_t length)
{
    uint16_t crc_expected;
    uint16_t crc_received;

    if ((slave_num >= UART_CHANNEL_COUNT) || (frame == NULL) || (length < 2u))
    {
        return;
    }

    if (frame[0] != MODBUS_DEFAULT_SLAVE_ADDRESS)
    {
        return;
    }

    if ((length >= 8u) &&
        (frame[1] == MODBUS_FUNC_WRITE_SINGLE_REG) &&
        (frame[2] == 0x00u) &&
        (frame[3] == 0x00u) &&
        (frame[4] == 0x12u) &&
        (frame[5] == 0x34u))
    {
        crc_expected = Modbus_CalculateCRC(frame, 6u);
        crc_received = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
        slave_boot_resp_state[slave_num] = (crc_expected == crc_received)
                                          ? SLAVE_BOOT_RESP_OK
                                          : SLAVE_BOOT_RESP_INVALID;
        return;
    }

    if (((frame[1] & 0x7Fu) == MODBUS_FUNC_WRITE_SINGLE_REG) && (length >= 5u))
    {
        slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_INVALID;
    }
}

/* Возвращает битовую маску для номера ведомого канала. */
static uint8_t Slave_GetChannelBit(uint8_t slave_num)
{
    if (slave_num >= UART_CHANNEL_COUNT)
    {
        return 0u;
    }

    return (uint8_t)(1u << slave_num);
}

/* Собирает маску ведомых каналов, по которым сейчас есть валидная Modbus-связь. */
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

/* Проверяет, что во flash сохранен пригодный slave-образ для рассылки ведомым контроллерам. */
static bool Slave_HasStoredImageReady(void)
{
    return (slave.os_in_flash != 0u) &&
           (slave.os_size_bytes > 0u) &&
           (slave.os_size_bytes <= FLASH_FW_STORAGE_SIZE) &&
           (slave.os_size_bytes <= SLAVE_FLASH_TARGET_SIZE_BYTES);
}

/* Копирует короткий отладочный текст в volatile-буфер без выхода за границы. */
static void Slave_CopyDebugText(volatile char *destination,
                                const char *source,
                                uint16_t destination_size)
{
    uint16_t index = 0u;

    if ((destination == NULL) || (destination_size == 0u))
    {
        return;
    }

    if (source == NULL)
    {
        source = "NULL";
    }

    while ((index + 1u) < destination_size)
    {
        char value = source[index];

        destination[index] = value;
        if (value == '\0')
        {
            return;
        }

        index++;
    }

    destination[index] = '\0';
}

/* Возвращает текстовое имя фазы автомата прошивки для отладки. */
static const char *Slave_GetPhaseText(SlaveUpdatePhase_t phase)
{
    switch (phase)
    {
        case SLAVE_UPD_IDLE: return "IDLE";
        case SLAVE_UPD_SEND_ENTER_BOOTLOADER: return "SEND_ENTER_BOOT";
        case SLAVE_UPD_WAIT_BOOTLOADER_ENTRY: return "WAIT_BOOT_ENTRY";
        case SLAVE_UPD_SEND_SYNC: return "SEND_SYNC";
        case SLAVE_UPD_WAIT_ACK_SYNC: return "WAIT_ACK_SYNC";
        case SLAVE_UPD_SEND_EXT_ERASE_CMD: return "SEND_EXT_ERASE_CMD";
        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD: return "WAIT_ACK_ERASE_CMD";
        case SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD: return "SEND_ERASE_PAYLOAD";
        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD: return "WAIT_ACK_ERASE_PAYLOAD";
        case SLAVE_UPD_PREPARE_BLOCK: return "PREPARE_BLOCK";
        case SLAVE_UPD_SEND_WRITE_CMD: return "SEND_WRITE_CMD";
        case SLAVE_UPD_WAIT_ACK_WRITE_CMD: return "WAIT_ACK_WRITE_CMD";
        case SLAVE_UPD_SEND_ADDRESS: return "SEND_ADDRESS";
        case SLAVE_UPD_WAIT_ACK_ADDRESS: return "WAIT_ACK_ADDRESS";
        case SLAVE_UPD_SEND_DATA: return "SEND_DATA";
        case SLAVE_UPD_WAIT_ACK_DATA: return "WAIT_ACK_DATA";
        case SLAVE_UPD_NEXT_BLOCK: return "NEXT_BLOCK";
        case SLAVE_UPD_SEND_GO_CMD: return "SEND_GO_CMD";
        case SLAVE_UPD_WAIT_ACK_GO_CMD: return "WAIT_ACK_GO_CMD";
        case SLAVE_UPD_SEND_GO_ADDRESS: return "SEND_GO_ADDRESS";
        case SLAVE_UPD_WAIT_ACK_GO_ADDRESS: return "WAIT_ACK_GO_ADDRESS";
        case SLAVE_UPD_DONE: return "DONE";
        case SLAVE_UPD_ERROR: return "ERROR";
        default: return "UNKNOWN_PHASE";
    }
}

/* Возвращает текстовое описание причины события или отказа прошивки. */
static const char *Slave_GetReasonText(uint8_t reason)
{
    switch (reason)
    {
        case SLAVE_UPDATE_DEBUG_REASON_NONE: return "NONE";
        case SLAVE_UPDATE_DEBUG_REASON_DISABLED: return "DISABLED";
        case SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_IMAGE: return "REQUEST_NO_IMAGE";
        case SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_TARGETS: return "REQUEST_NO_TARGETS";
        case SLAVE_UPDATE_DEBUG_REASON_REQUEST_BUSY: return "REQUEST_BUSY";
        case SLAVE_UPDATE_DEBUG_REASON_START_NO_IMAGE: return "START_NO_IMAGE";
        case SLAVE_UPDATE_DEBUG_REASON_START_DISCONNECTED: return "START_DISCONNECTED";
        case SLAVE_UPDATE_DEBUG_REASON_INVALID_SIZE: return "INVALID_SIZE";
        case SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED: return "UART_TX_FAILED";
        case SLAVE_UPDATE_DEBUG_REASON_TIMEOUT: return "TIMEOUT";
        case SLAVE_UPDATE_DEBUG_REASON_NACK: return "NACK";
        case SLAVE_UPDATE_DEBUG_REASON_RETRY_LIMIT: return "RETRY_LIMIT";
        case SLAVE_UPDATE_DEBUG_REASON_CANCELLED: return "CANCELLED";
        case SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_TIMEOUT: return "BOOT_RESP_TIMEOUT";
        case SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_INVALID: return "BOOT_RESP_INVALID";
        default: return "UNKNOWN_REASON";
    }
}

/* Очищает сохраненный снимок последней ошибки прошивки ведомого контроллера. */
static void Slave_ClearFailureSnapshot(void)
{
    g_slave_update_debug.fail_valid = 0u;
    g_slave_update_debug.fail_slave_num = SLAVE_UPDATE_INVALID_INDEX;
    g_slave_update_debug.fail_phase = (uint8_t)SLAVE_UPD_IDLE;
    g_slave_update_debug.fail_event = SLAVE_UPDATE_DEBUG_EVENT_NONE;
    g_slave_update_debug.fail_terminal_reason = SLAVE_UPDATE_DEBUG_REASON_NONE;
    g_slave_update_debug.fail_detail_reason = SLAVE_UPDATE_DEBUG_REASON_NONE;
    g_slave_update_debug.fail_response = 0u;
    g_slave_update_debug.fail_retry_count = 0u;
    g_slave_update_debug.fail_ack = 0u;
    g_slave_update_debug.fail_tick = 0u;
    g_slave_update_debug.fail_last_command_tick = 0u;
    g_slave_update_debug.fail_bytes_written = 0u;
    g_slave_update_debug.fail_total_bytes = 0u;
    g_slave_update_debug.fail_target_address = 0u;
    g_slave_update_debug.fail_payload_size = 0u;
    g_slave_update_debug.fail_padded_size = 0u;
    g_slave_update_debug.fail_erase_next_page = 0u;
    g_slave_update_debug.fail_erase_last_page = 0u;
    g_slave_update_debug.fail_erase_pages_in_block = 0u;
    Slave_CopyDebugText(g_slave_update_debug.fail_phase_text, "NONE", SLAVE_UPDATE_DEBUG_TEXT_SIZE);
    Slave_CopyDebugText(g_slave_update_debug.fail_reason_text, "NONE", SLAVE_UPDATE_DEBUG_TEXT_SIZE);
    Slave_CopyDebugText(g_slave_update_debug.fail_detail_text, "NONE", SLAVE_UPDATE_DEBUG_TEXT_SIZE);
}

/* Фиксирует подробный снимок отказа до очистки рабочего контекста канала. */
static void Slave_SaveFailureSnapshot(uint8_t slave_num,
                                      SlaveUpdatePhase_t failed_phase,
                                      uint8_t terminal_reason,
                                      uint8_t detail_reason,
                                      uint8_t response,
                                      const SlaveUpdateContext_t *ctx,
                                      uint32_t current_tick)
{
    /*
     * Этот снимок не живой, а посмертный: он остается после завершения процесса,
     * когда ctx уже очищен и live_phase снова IDLE. Так в отладчике сразу видно,
     * где именно все упало, без угадывания по кольцевому trace.
     */
    g_slave_update_debug.fail_valid = 1u;
    g_slave_update_debug.fail_slave_num = slave_num;
    g_slave_update_debug.fail_phase = (uint8_t)failed_phase;
    g_slave_update_debug.fail_event = SLAVE_UPDATE_DEBUG_EVENT_ERROR;
    g_slave_update_debug.fail_terminal_reason = terminal_reason;
    g_slave_update_debug.fail_detail_reason = detail_reason;
    g_slave_update_debug.fail_response = response;
    g_slave_update_debug.fail_retry_count = (ctx != NULL) ? ctx->retry_count : 0u;
    g_slave_update_debug.fail_ack = (slave_num < UART_CHANNEL_COUNT) ? slave.device[slave_num].ack : 0u;
    g_slave_update_debug.fail_tick = current_tick;
    g_slave_update_debug.fail_last_command_tick = (ctx != NULL) ? ctx->last_command_tick : 0u;
    g_slave_update_debug.fail_bytes_written = (ctx != NULL) ? ctx->bytes_written : 0u;
    g_slave_update_debug.fail_total_bytes = (ctx != NULL) ? ctx->total_bytes : 0u;
    g_slave_update_debug.fail_target_address = (ctx != NULL) ? ctx->target_address : 0u;
    g_slave_update_debug.fail_payload_size = (ctx != NULL) ? ctx->payload_size : 0u;
    g_slave_update_debug.fail_padded_size = (ctx != NULL) ? ctx->padded_size : 0u;
    g_slave_update_debug.fail_erase_next_page = (ctx != NULL) ? ctx->erase_next_page : 0u;
    g_slave_update_debug.fail_erase_last_page = (ctx != NULL) ? ctx->erase_last_page : 0u;
    g_slave_update_debug.fail_erase_pages_in_block = (ctx != NULL) ? ctx->erase_pages_in_block : 0u;

    Slave_CopyDebugText(g_slave_update_debug.fail_phase_text,
                        Slave_GetPhaseText(failed_phase),
                        SLAVE_UPDATE_DEBUG_TEXT_SIZE);
    Slave_CopyDebugText(g_slave_update_debug.fail_reason_text,
                        Slave_GetReasonText(terminal_reason),
                        SLAVE_UPDATE_DEBUG_TEXT_SIZE);
    Slave_CopyDebugText(g_slave_update_debug.fail_detail_text,
                        Slave_GetReasonText(detail_reason),
                        SLAVE_UPDATE_DEBUG_TEXT_SIZE);
}

/* Сбрасывает batch-состояние очереди прошивки ведомых каналов. */
static void Slave_ResetBatchState(void)
{
    slave_update_pending_mask = 0u;
    slave_update_success_mask = 0u;
    slave_update_error_mask = 0u;
    slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
    Slave_ClearFailureSnapshot();
}

/* Обновляет живой отладочный снимок текущего состояния автомата прошивки. */
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
    g_slave_update_debug.live_payload_size = (ctx != NULL) ? ctx->payload_size : 0u;
    g_slave_update_debug.live_padded_size = (ctx != NULL) ? ctx->padded_size : 0u;
}

/* Записывает одно событие автомата прошивки в debug-поля и кольцевую трассу. */
static void Slave_RecordDebugEvent(uint8_t slave_num,
                                   uint8_t event_code,
                                   SlaveUpdatePhase_t phase,
                                   uint8_t reason,
                                   uint8_t response,
                                   const SlaveUpdateContext_t *ctx,
                                   uint32_t current_tick)
{
    uint32_t write_index = g_slave_update_debug.trace_write_index;
    uint32_t next_index = (write_index + 1u) % SLAVE_UPDATE_TRACE_DEPTH;
    uint32_t sequence = g_slave_update_debug.event_counter + 1u;
    volatile SlaveUpdateTraceEntry_t *entry = &g_slave_update_trace[write_index];

    g_slave_update_debug.event_counter = sequence;
    g_slave_update_debug.last_event = event_code;
    g_slave_update_debug.last_reason = reason;
    g_slave_update_debug.last_response = response;
    g_slave_update_debug.last_slave_num = slave_num;
    g_slave_update_debug.last_phase = (uint8_t)phase;
    g_slave_update_debug.trace_write_index = next_index;
    if (g_slave_update_debug.trace_count < SLAVE_UPDATE_TRACE_DEPTH)
    {
        g_slave_update_debug.trace_count++;
    }
    else
    {
        g_slave_update_debug.trace_wrap_count++;
    }

    entry->sequence = sequence;
    entry->tick = current_tick;
    entry->bytes_written = (ctx != NULL) ? ctx->bytes_written : 0u;
    entry->total_bytes = (ctx != NULL) ? ctx->total_bytes : 0u;
    entry->target_address = (ctx != NULL) ? ctx->target_address : 0u;
    entry->payload_size = (ctx != NULL) ? ctx->payload_size : 0u;
    entry->padded_size = (ctx != NULL) ? ctx->padded_size : 0u;
    entry->slave_num = slave_num;
    entry->phase = (uint8_t)phase;
    entry->event = event_code;
    entry->reason = reason;
    entry->response = response;
    entry->retry_count = (ctx != NULL) ? ctx->retry_count : 0u;

    Slave_UpdateDebugSnapshot(current_tick);
}

/* Записывает событие перехода фазы для конкретного рабочего контекста. */
static void Slave_RecordPhaseTrace(const SlaveUpdateContext_t *ctx, uint32_t current_tick)
{
    if (ctx == NULL)
    {
        return;
    }

    Slave_RecordDebugEvent(ctx->slave_number,
                           SLAVE_UPDATE_DEBUG_EVENT_PHASE,
                           ctx->phase,
                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                           0u,
                           ctx,
                           current_tick);
}

/* Завершает активную прошивку канала и переносит результат в маски success/error. */
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

/* Помечает набор каналов как завершившийся ошибкой без запуска передачи. */
static void Slave_MarkMaskedChannelsAsError(uint8_t mask)
{
    slave_update_error_mask |= mask;
}

/* Выбирает следующий ожидающий канал и подготавливает его к запуску прошивки. */
static void Slave_StartNextPendingUpdate(void)
{
    if (!Slave_HasStoredImageReady())
    {
        Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                               SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                               SLAVE_UPD_IDLE,
                               SLAVE_UPDATE_DEBUG_REASON_START_NO_IMAGE,
                               0u,
                               NULL,
                               HAL_GetTick());
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
                Slave_RecordDebugEvent(slave_num,
                                       SLAVE_UPDATE_DEBUG_EVENT_REJECTED,
                                       SLAVE_UPD_IDLE,
                                       SLAVE_UPDATE_DEBUG_REASON_START_DISCONNECTED,
                                       0u,
                                       NULL,
                                       HAL_GetTick());
                slave_update_error_mask |= channel_bit;
                continue;
            }

            slave_update_active_slave = slave_num;
            slave.device[slave_num].need_update = 1u;
            Slave_RecordDebugEvent(slave_num,
                                   SLAVE_UPDATE_DEBUG_EVENT_START,
                                   SLAVE_UPD_IDLE,
                                   SLAVE_UPDATE_DEBUG_REASON_NONE,
                                   0u,
                                   NULL,
                                   HAL_GetTick());
            Slave_UpdateDebugSnapshot(HAL_GetTick());
            return;
        }
    }

    slave_update_active_slave = SLAVE_UPDATE_INVALID_INDEX;
    Slave_UpdateDebugSnapshot(HAL_GetTick());
}

/* Валидирует маску запроса и ставит выбранные ведомые каналы в очередь прошивки. */
static bool Slave_RequestUpdateMask(uint8_t request_mask)
{
    if ((SLAVE_FW_SEND_ENABLE == 0u) || !Slave_HasStoredImageReady())
    {
        Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                               SLAVE_UPDATE_DEBUG_EVENT_REJECTED,
                               SLAVE_UPD_IDLE,
                               (SLAVE_FW_SEND_ENABLE == 0u)
                                   ? SLAVE_UPDATE_DEBUG_REASON_DISABLED
                                   : SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_IMAGE,
                               0u,
                               NULL,
                               HAL_GetTick());
        return false;
    }

    request_mask &= Slave_GetConnectedMask();
    if (request_mask == 0u)
    {
        Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                               SLAVE_UPDATE_DEBUG_EVENT_REJECTED,
                               SLAVE_UPD_IDLE,
                               SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_TARGETS,
                               0u,
                               NULL,
                               HAL_GetTick());
        return false;
    }

    if (Slave_IsAnyUpdateRunning())
    {
        Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                               SLAVE_UPDATE_DEBUG_EVENT_REJECTED,
                               SLAVE_UPD_IDLE,
                               SLAVE_UPDATE_DEBUG_REASON_REQUEST_BUSY,
                               0u,
                               NULL,
                               HAL_GetTick());
        return false;
    }

    taskENTER_CRITICAL();
    Slave_ResetBatchState();
    slave_update_pending_mask = request_mask;
    slave_update_auto_pending = 0u;
    Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                           SLAVE_UPDATE_DEBUG_EVENT_REQUEST,
                           SLAVE_UPD_IDLE,
                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                           0u,
                           NULL,
                           HAL_GetTick());
    Slave_UpdateDebugSnapshot(HAL_GetTick());
    taskEXIT_CRITICAL();

    return true;
}

/* Проверяет, является ли фаза ожиданием ACK от системного bootloader. */
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

/* Возвращает таймаут ожидания для текущей фазы автомата прошивки. */
static uint32_t Slave_GetPhaseTimeoutMs(SlaveUpdatePhase_t phase)
{
    if (phase == SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD)
    {
        return SLAVE_UPDATE_ERASE_TIMEOUT_MS;
    }

    return SLAVE_UPDATE_TIMEOUT_MS;
}

/* Определяет фазу, с которой нужно повторить команду после NACK или timeout. */
static SlaveUpdatePhase_t Slave_GetRetryPhase(SlaveUpdatePhase_t phase)
{
    switch (phase)
    {
        case SLAVE_UPD_WAIT_BOOTLOADER_ENTRY:
            return SLAVE_UPD_SEND_ENTER_BOOTLOADER;

        case SLAVE_UPD_WAIT_ACK_SYNC:
            return SLAVE_UPD_SEND_ENTER_BOOTLOADER;

        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD:
            return SLAVE_UPD_SEND_EXT_ERASE_CMD;

        case SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD:
            /*
             * После NACK загрузчик возвращается к ожиданию новой команды.
             * Поэтому повторяем весь Extended Erase для того же блока страниц,
             * а не шлем payload отдельно в никуда.
             */
            return SLAVE_UPD_SEND_EXT_ERASE_CMD;

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

/* Планирует повтор команды или переводит канал в ошибку при исчерпании попыток. */
static void Slave_ScheduleRetryOrError(SlaveUpdateContext_t *ctx,
                                       uint8_t reason,
                                       uint32_t current_tick)
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
        Slave_RecordDebugEvent(ctx->slave_number,
                               SLAVE_UPDATE_DEBUG_EVENT_RETRY,
                               ctx->phase,
                               reason,
                               0u,
                               ctx,
                               current_tick);
        Slave_RecordPhaseTrace(ctx, current_tick);
    }
    else
    {
        SlaveUpdatePhase_t failed_phase = ctx->phase;
        uint8_t response = slave.device[ctx->slave_number].ack;

        Slave_SaveFailureSnapshot(ctx->slave_number,
                                  failed_phase,
                                  SLAVE_UPDATE_DEBUG_REASON_RETRY_LIMIT,
                                  reason,
                                  response,
                                  ctx,
                                  current_tick);
        ctx->phase = SLAVE_UPD_ERROR;
        Slave_RecordDebugEvent(ctx->slave_number,
                               SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                               failed_phase,
                               SLAVE_UPDATE_DEBUG_REASON_RETRY_LIMIT,
                               response,
                               ctx,
                               current_tick);
    }
}

/* Готовит следующий блок данных slave-образа для команды Write Memory. */
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

/* Формирует payload команды Extended Erase для очередного блока flash-страниц. */
static bool Slave_PrepareExtErasePayload(SlaveUpdateContext_t *ctx,
                                         uint8_t *payload,
                                         uint16_t payload_capacity,
                                         uint16_t *payload_length)
{
    uint16_t pages_remaining;
    uint16_t pages_in_block;
    uint16_t required_length;
    uint16_t page_count_minus_one;
    uint8_t checksum = 0u;

    if ((ctx == NULL) || (payload == NULL) || (payload_length == NULL))
    {
        return false;
    }

    if ((SLAVE_EXT_ERASE_PAGES_PER_CMD == 0u) ||
        (ctx->erase_next_page > ctx->erase_last_page))
    {
        return false;
    }

    /*
     * AN3155 Extended Erase:
     * - первые 2 байта: N = количество страниц/секторов минус 1, MSB first;
     * - дальше по 2 байта на каждый номер страницы/сектора, тоже MSB first;
     * - последний байт: XOR всех предыдущих байтов payload.
     *
     * Для STM32L051C6 это реальные flash-страницы по 128 байт: 0..255.
     * Стираем весь flash слейва постранично, потому что mass erase через 0xFFFF
     * этот загрузчик не принимает.
     */
    pages_remaining = (uint16_t)(ctx->erase_last_page - ctx->erase_next_page + 1u);
    pages_in_block = (pages_remaining > SLAVE_EXT_ERASE_PAGES_PER_CMD)
                   ? SLAVE_EXT_ERASE_PAGES_PER_CMD
                   : pages_remaining;
    required_length = (uint16_t)(2u + (2u * pages_in_block) + 1u);

    if (payload_capacity < required_length)
    {
        return false;
    }

    page_count_minus_one = (uint16_t)(pages_in_block - 1u);
    payload[0] = (uint8_t)((page_count_minus_one >> 8) & 0xFFu);
    payload[1] = (uint8_t)(page_count_minus_one & 0xFFu);
    checksum = (uint8_t)(payload[0] ^ payload[1]);

    for (uint16_t index = 0u; index < pages_in_block; ++index)
    {
        uint16_t page = (uint16_t)(ctx->erase_next_page + index);
        uint16_t offset = (uint16_t)(2u + (2u * index));

        payload[offset] = (uint8_t)((page >> 8) & 0xFFu);
        payload[offset + 1u] = (uint8_t)(page & 0xFFu);
        checksum ^= payload[offset];
        checksum ^= payload[offset + 1u];
    }

    payload[required_length - 1u] = checksum;
    ctx->erase_pages_in_block = pages_in_block;
    *payload_length = required_length;
    return true;
}

/* Отменяет все ожидающие и активные прошивки ведомых контроллеров. */
void Slave_CancelPendingUpdates(void)
{
    Slave_RecordDebugEvent(slave_update_active_slave,
                           SLAVE_UPDATE_DEBUG_EVENT_CANCEL,
                           SLAVE_UPD_IDLE,
                           SLAVE_UPDATE_DEBUG_REASON_CANCELLED,
                           0u,
                           NULL,
                           HAL_GetTick());

    taskENTER_CRITICAL();

    for (uint8_t slave_num = 0u; slave_num < UART_CHANNEL_COUNT; ++slave_num)
    {
        slave.device[slave_num].need_update = 0u;
        slave.device[slave_num].ack = 0u;
        slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_NONE;
        memset(&slave_update_ctx[slave_num], 0, sizeof(slave_update_ctx[slave_num]));
    }

    Slave_ResetBatchState();
    slave_update_auto_pending = 0u;
    Slave_UpdateDebugSnapshot(HAL_GetTick());
    taskEXIT_CRITICAL();
}

/* Проверяет, выполняется ли сейчас хотя бы одна slave-прошивка или есть очередь. */
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

/* Запрашивает прошивку одного конкретного ведомого канала. */
bool Slave_RequestSingleUpdate(uint8_t slave_num)
{
    uint8_t request_mask = Slave_GetChannelBit(slave_num);
    return Slave_RequestUpdateMask(request_mask);
}

/* Запрашивает прошивку всех сейчас подключенных ведомых каналов. */
bool Slave_RequestConnectedUpdate(void)
{
    return Slave_RequestUpdateMask(0xFFu);
}

/* Ставит автоматический запуск прошивки подключенных каналов после завершения приема образа. */
bool Slave_RequestAutoConnectedUpdate(void)
{
    if ((SLAVE_FW_SEND_ENABLE == 0u) || !Slave_HasStoredImageReady())
    {
        Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                               SLAVE_UPDATE_DEBUG_EVENT_REJECTED,
                               SLAVE_UPD_IDLE,
                               (SLAVE_FW_SEND_ENABLE == 0u)
                                   ? SLAVE_UPDATE_DEBUG_REASON_DISABLED
                                   : SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_IMAGE,
                               0u,
                               NULL,
                               HAL_GetTick());
        return false;
    }

    taskENTER_CRITICAL();
    slave_update_auto_pending = 1u;
    Slave_RecordDebugEvent(SLAVE_UPDATE_INVALID_INDEX,
                           SLAVE_UPDATE_DEBUG_EVENT_AUTO,
                           SLAVE_UPD_IDLE,
                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                           0u,
                           NULL,
                           HAL_GetTick());
    Slave_UpdateDebugSnapshot(HAL_GetTick());
    taskEXIT_CRITICAL();

    return true;
}

/* Проверяет, занят ли конкретный ведомый канал процессом прошивки. */
bool Slave_IsUpdateActive(uint8_t slave_num)
{
    if (slave_num >= UART_CHANNEL_COUNT)
    {
        return false;
    }

    return (slave.device[slave_num].need_update != 0u) ||
           (slave_update_ctx[slave_num].phase != SLAVE_UPD_IDLE);
}

/* Выполняет один шаг автомата фоновой прошивки ведомых контроллеров. */
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
                (ctx->total_bytes > SLAVE_FLASH_TARGET_SIZE_BYTES) ||
                (SLAVE_FLASH_ERASE_FIRST_PAGE > SLAVE_FLASH_ERASE_LAST_PAGE))
            {
                ctx->phase = SLAVE_UPD_ERROR;
                Slave_RecordDebugEvent(slave_num,
                                       SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                                       ctx->phase,
                                       SLAVE_UPDATE_DEBUG_REASON_INVALID_SIZE,
                                       0u,
                                       ctx,
                                       current_tick);
            }
            else
            {
                ctx->app_start_address = SLAVE_FLASH_TARGET_START_ADDR;
                ctx->target_address = ctx->app_start_address;
                ctx->erase_next_page = SLAVE_FLASH_ERASE_FIRST_PAGE;
                ctx->erase_last_page = SLAVE_FLASH_ERASE_LAST_PAGE;
                ctx->erase_pages_in_block = 0u;
                ctx->phase = SLAVE_UPD_SEND_ENTER_BOOTLOADER;
                Slave_RecordPhaseTrace(ctx, current_tick);
            }
        }

        if (Slave_IsAckWaitPhase(ctx->phase) &&
            ((current_tick - ctx->last_command_tick) > Slave_GetPhaseTimeoutMs(ctx->phase)))
        {
            Slave_RecordDebugEvent(ctx->slave_number,
                                   SLAVE_UPDATE_DEBUG_EVENT_TIMEOUT,
                                   ctx->phase,
                                   SLAVE_UPDATE_DEBUG_REASON_TIMEOUT,
                                   slave.device[ctx->slave_number].ack,
                                   ctx,
                                   current_tick);
            Slave_ScheduleRetryOrError(ctx,
                                       SLAVE_UPDATE_DEBUG_REASON_TIMEOUT,
                                       current_tick);
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
                slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_NONE;
                ctx->boot_resp_received = 0u;

                if (Slave_SendBootloaderCommand(slave_num, modbus_cmd, sizeof(modbus_cmd)))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_BOOTLOADER_ENTRY;
                    ctx->last_command_tick = current_tick;
                    slave.device[slave_num].ack = 0u;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_BOOTLOADER_ENTRY:
            {
                uint8_t boot_resp_state = slave_boot_resp_state[slave_num];

                if ((ctx->boot_resp_received == 0u) && (boot_resp_state == SLAVE_BOOT_RESP_OK))
                {
                    ctx->boot_resp_received = 1u;
                    ctx->retry_count = 0u;
                    slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_NONE;
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_BOOT_RESP,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           MODBUS_FUNC_WRITE_SINGLE_REG,
                                           ctx,
                                           current_tick);
                }
                else if (boot_resp_state == SLAVE_BOOT_RESP_INVALID)
                {
                    slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_NONE;
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_BOOT_RESP,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_INVALID,
                                           0u,
                                           ctx,
                                           current_tick);
                    Slave_ScheduleRetryOrError(ctx,
                                               SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_INVALID,
                                               current_tick);
                    break;
                }

                if ((ctx->boot_resp_received == 0u) &&
                    ((current_tick - ctx->last_command_tick) > SLAVE_BOOTLOADER_RESP_TIMEOUT_MS))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TIMEOUT,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_TIMEOUT,
                                           0u,
                                           ctx,
                                           current_tick);
                    Slave_ScheduleRetryOrError(ctx,
                                               SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_TIMEOUT,
                                               current_tick);
                    break;
                }

                if (ctx->boot_resp_received != 0u &&
                    ((current_tick - ctx->last_command_tick) >= SLAVE_BOOTLOADER_ENTRY_DELAY_MS))
                {
                    ctx->phase = SLAVE_UPD_SEND_SYNC;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                break;
            }

            case SLAVE_UPD_SEND_SYNC:
            {
                const uint8_t sync_cmd = BOOTLOADER_SYNC;

                if (Slave_SendBootloaderCommand(slave_num, &sync_cmd, 1u))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_SYNC;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_SYNC:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_EXT_ERASE_CMD;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_EXT_ERASE_CMD:
            {
                const uint8_t erase_cmd[2] = {BOOTLOADER_EXT_ERASE, (uint8_t)(BOOTLOADER_EXT_ERASE ^ 0xFFu)};

                if (Slave_SendBootloaderCommand(slave_num, erase_cmd, sizeof(erase_cmd)))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_EXT_ERASE_CMD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
                    }
                }
                break;

            case SLAVE_UPD_SEND_EXT_ERASE_PAYLOAD:
            {
                uint8_t erase_payload[SLAVE_EXT_ERASE_PAYLOAD_MAX_SIZE];
                uint16_t erase_payload_length = 0u;

                /*
                 * Стираем весь программный flash STM32L051C6: страницы 0..255.
                 * В одну команду кладем не весь список, а блок страниц, чтобы
                 * не строить огромный пакет и проще понимать, где все упало.
                 */
                if (!Slave_PrepareExtErasePayload(ctx,
                                                  erase_payload,
                                                  sizeof(erase_payload),
                                                  &erase_payload_length))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_INVALID_SIZE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                    break;
                }

                if (Slave_SendBootloaderCommand(slave_num, erase_payload, erase_payload_length))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_EXT_ERASE_PAYLOAD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        /*
                         * ACK пришел уже после фактического стирания блока.
                         * Только теперь сдвигаем указатель, чтобы при NACK/timeout
                         * повторить ровно тот же блок страниц.
                         */
                        ctx->erase_next_page = (uint16_t)(ctx->erase_next_page + ctx->erase_pages_in_block);
                        ctx->erase_pages_in_block = 0u;
                        ctx->phase = (ctx->erase_next_page <= ctx->erase_last_page)
                                   ? SLAVE_UPD_SEND_EXT_ERASE_CMD
                                   : SLAVE_UPD_PREPARE_BLOCK;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
                    }
                }
                break;

            case SLAVE_UPD_PREPARE_BLOCK:
                if (!Slave_PrepareNextDataBlock(ctx))
                {
                    ctx->phase = SLAVE_UPD_ERROR;
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_INVALID_SIZE,
                                           0u,
                                           ctx,
                                           current_tick);
                }
                else
                {
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                break;

            case SLAVE_UPD_SEND_WRITE_CMD:
            {
                const uint8_t write_cmd[2] = {BOOTLOADER_WRITE, (uint8_t)(BOOTLOADER_WRITE ^ 0xFFu)};

                if (Slave_SendBootloaderCommand(slave_num, write_cmd, sizeof(write_cmd)))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_WRITE_CMD;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_WRITE_CMD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_ADDRESS;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
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
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_ADDRESS;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_ADDRESS:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_DATA;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
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
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_DATA;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_DATA:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->bytes_written += ctx->payload_size;
                        ctx->target_address += ctx->padded_size;
                        ctx->phase = SLAVE_UPD_NEXT_BLOCK;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
                    }
                }
                break;

            case SLAVE_UPD_NEXT_BLOCK:
                ctx->phase = (ctx->bytes_written < ctx->total_bytes)
                           ? SLAVE_UPD_PREPARE_BLOCK
                           : SLAVE_UPD_SEND_GO_CMD;
                Slave_RecordPhaseTrace(ctx, current_tick);
                break;

            case SLAVE_UPD_SEND_GO_CMD:
            {
                const uint8_t go_cmd[2] = {BOOTLOADER_GO, (uint8_t)(BOOTLOADER_GO ^ 0xFFu)};

                if (Slave_SendBootloaderCommand(slave_num, go_cmd, sizeof(go_cmd)))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_GO_CMD;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_GO_CMD:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_SEND_GO_ADDRESS;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
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
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_OK,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_NONE,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_WAIT_ACK_GO_ADDRESS;
                    ctx->last_command_tick = current_tick;
                    Slave_RecordPhaseTrace(ctx, current_tick);
                }
                else
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL,
                                           ctx->phase,
                                           SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED,
                                           0u,
                                           ctx,
                                           current_tick);
                    ctx->phase = SLAVE_UPD_ERROR;
                }
                break;
            }

            case SLAVE_UPD_WAIT_ACK_GO_ADDRESS:
                if (Slave_TakeBootloaderResponse(slave_num, &response))
                {
                    Slave_RecordDebugEvent(slave_num,
                                           SLAVE_UPDATE_DEBUG_EVENT_ACK_RX,
                                           ctx->phase,
                                           (response == BOOTLOADER_ACK)
                                               ? SLAVE_UPDATE_DEBUG_REASON_NONE
                                               : SLAVE_UPDATE_DEBUG_REASON_NACK,
                                           response,
                                           ctx,
                                           current_tick);
                    if (response == BOOTLOADER_ACK)
                    {
                        ctx->phase = SLAVE_UPD_DONE;
                        ctx->retry_count = 0u;
                        Slave_RecordPhaseTrace(ctx, current_tick);
                    }
                    else
                    {
                        Slave_ScheduleRetryOrError(ctx,
                                                   SLAVE_UPDATE_DEBUG_REASON_NACK,
                                                   current_tick);
                    }
                }
                break;

            case SLAVE_UPD_DONE:
                Slave_FinishActiveUpdate(slave_num, true);
                Slave_RecordDebugEvent(slave_num,
                                       SLAVE_UPDATE_DEBUG_EVENT_DONE,
                                       ctx->phase,
                                       SLAVE_UPDATE_DEBUG_REASON_NONE,
                                       0u,
                                       ctx,
                                       current_tick);
                slave.device[slave_num].need_update = 0u;
                slave.device[slave_num].ack = 0u;
                slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_NONE;
                memset(ctx, 0, sizeof(*ctx));
                break;

            case SLAVE_UPD_ERROR:
            {
                SlaveUpdatePhase_t report_phase;
                uint8_t report_reason;
                uint8_t report_response;

                if ((g_slave_update_debug.fail_valid == 0u) ||
                    (g_slave_update_debug.fail_slave_num != slave_num))
                {
                    SlaveUpdatePhase_t failed_phase = (SlaveUpdatePhase_t)g_slave_update_debug.last_phase;
                    uint8_t failed_reason = g_slave_update_debug.last_reason;
                    uint8_t failed_response = g_slave_update_debug.last_response;

                    /*
                     * Если ошибка была выставлена напрямую через ctx->phase = ERROR,
                     * сохраняем последнюю осмысленную фазу из debug-события перед
                     * очисткой контекста. Иначе после завершения опять будет чертов
                     * IDLE/ERROR без ответа, где именно все умерло.
                     */
                    Slave_SaveFailureSnapshot(slave_num,
                                              failed_phase,
                                              failed_reason,
                                              failed_reason,
                                              failed_response,
                                              ctx,
                                              current_tick);
                }

                report_phase = (SlaveUpdatePhase_t)g_slave_update_debug.fail_phase;
                report_reason = g_slave_update_debug.fail_terminal_reason;
                report_response = g_slave_update_debug.fail_response;

                Slave_FinishActiveUpdate(slave_num, false);
                Slave_RecordDebugEvent(slave_num,
                                       SLAVE_UPDATE_DEBUG_EVENT_ERROR,
                                       report_phase,
                                       report_reason,
                                       report_response,
                                       ctx,
                                       current_tick);
                slave.device[slave_num].need_update = 0u;
                slave.device[slave_num].ack = 0u;
                slave_boot_resp_state[slave_num] = SLAVE_BOOT_RESP_NONE;
                memset(ctx, 0, sizeof(*ctx));
                break;
            }

            case SLAVE_UPD_IDLE:
            default:
                break;
        }
    }

    Slave_UpdateDebugSnapshot(current_tick);
}

/* Основная задача FreeRTOS для фоновой прошивки ведомых контроллеров. */
void Slave_UpdateTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        Slave_UpdateProcess();
        osDelay(50);
    }
}
