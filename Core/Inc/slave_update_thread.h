#ifndef SLAVE_UPDATE_THREAD_H
#define SLAVE_UPDATE_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "uart.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define SLAVE_UPDATE_TIMEOUT_MS           15000u
#define SLAVE_UPDATE_ERASE_TIMEOUT_MS     120000u
#define SLAVE_UPDATE_RETRY_MAX            5u
#define SLAVE_FW_BLOCK_SIZE               256u
#define SLAVE_FW_SEND_ENABLE              1u
#define SLAVE_AUTO_UPDATE_ENABLE          1u
#define SLAVE_BOOTLOADER_ENTRY_DELAY_MS   5000u
#define SLAVE_BOOTLOADER_RESP_TIMEOUT_MS  1500u

/*
 * Профиль прошиваемого slave: ST STM32L051C6.
 *
 * Это не карта flash master-контроллера. Нельзя, блядь, брать сюда
 * FLASH_APP_START_ADDR от STM32F427, иначе ROM bootloader L051 закономерно
 * отвечает NACK на адрес 0x08040000.
 */
#define SLAVE_FLASH_TARGET_START_ADDR     0x08000000UL
#define SLAVE_FLASH_TARGET_SIZE_BYTES     (32u * 1024u)
#define SLAVE_FLASH_PAGE_SIZE_BYTES       128u
#define SLAVE_FLASH_ERASE_FIRST_PAGE      0u
#define SLAVE_FLASH_ERASE_LAST_PAGE       ((SLAVE_FLASH_TARGET_SIZE_BYTES / SLAVE_FLASH_PAGE_SIZE_BYTES) - 1u)

#define BOOTLOADER_ACK                    0x79u
#define BOOTLOADER_NACK                   0x1Fu
#define BOOTLOADER_SYNC                   0x7Fu
#define BOOTLOADER_EXT_ERASE              0x44u
#define BOOTLOADER_GO                     0x21u
#define BOOTLOADER_WRITE                  0x31u

#define SLAVE_UPDATE_TRACE_DEPTH          64u
#define SLAVE_UPDATE_DEBUG_TEXT_SIZE      32u

#define SLAVE_UPDATE_DEBUG_EVENT_NONE     0u
#define SLAVE_UPDATE_DEBUG_EVENT_DONE     1u
#define SLAVE_UPDATE_DEBUG_EVENT_ERROR    2u
#define SLAVE_UPDATE_DEBUG_EVENT_REQUEST  3u
#define SLAVE_UPDATE_DEBUG_EVENT_START    4u
#define SLAVE_UPDATE_DEBUG_EVENT_PHASE    5u
#define SLAVE_UPDATE_DEBUG_EVENT_TX_OK    6u
#define SLAVE_UPDATE_DEBUG_EVENT_TX_FAIL  7u
#define SLAVE_UPDATE_DEBUG_EVENT_ACK_RX   8u
#define SLAVE_UPDATE_DEBUG_EVENT_TIMEOUT  9u
#define SLAVE_UPDATE_DEBUG_EVENT_RETRY    10u
#define SLAVE_UPDATE_DEBUG_EVENT_CANCEL   11u
#define SLAVE_UPDATE_DEBUG_EVENT_REJECTED 12u
#define SLAVE_UPDATE_DEBUG_EVENT_AUTO     13u
#define SLAVE_UPDATE_DEBUG_EVENT_BOOT_RESP 14u

#define SLAVE_UPDATE_DEBUG_REASON_NONE                0u
#define SLAVE_UPDATE_DEBUG_REASON_DISABLED            1u
#define SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_IMAGE    2u
#define SLAVE_UPDATE_DEBUG_REASON_REQUEST_NO_TARGETS  3u
#define SLAVE_UPDATE_DEBUG_REASON_REQUEST_BUSY        4u
#define SLAVE_UPDATE_DEBUG_REASON_START_NO_IMAGE      5u
#define SLAVE_UPDATE_DEBUG_REASON_START_DISCONNECTED  6u
#define SLAVE_UPDATE_DEBUG_REASON_INVALID_SIZE        7u
#define SLAVE_UPDATE_DEBUG_REASON_UART_TX_FAILED      8u
#define SLAVE_UPDATE_DEBUG_REASON_TIMEOUT            9u
#define SLAVE_UPDATE_DEBUG_REASON_NACK               10u
#define SLAVE_UPDATE_DEBUG_REASON_RETRY_LIMIT        11u
#define SLAVE_UPDATE_DEBUG_REASON_CANCELLED          12u
#define SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_TIMEOUT  13u
#define SLAVE_UPDATE_DEBUG_REASON_BOOT_RESP_INVALID  14u

typedef struct
{
    volatile uint8_t ack;
    volatile uint8_t need_update;
} SlaveDevice_t;

typedef struct
{
    SlaveDevice_t device[UART_CHANNEL_COUNT];
    volatile uint8_t os_in_flash;
    volatile uint32_t os_size_bytes;
} SlaveSystem_t;

typedef struct
{
    volatile uint32_t event_counter;
    volatile uint8_t last_event;
    volatile uint8_t last_reason;
    volatile uint8_t last_response;
    volatile uint8_t last_slave_num;
    volatile uint8_t last_phase;
    volatile uint8_t live_slave_num;
    volatile uint8_t live_phase;
    volatile uint8_t live_retry_count;
    volatile uint8_t live_ack;
    volatile uint8_t auto_pending;
    volatile uint32_t last_tick;
    volatile uint32_t live_last_command_tick;
    volatile uint32_t live_bytes_written;
    volatile uint32_t live_total_bytes;
    volatile uint32_t live_target_address;
    volatile uint16_t live_payload_size;
    volatile uint16_t live_padded_size;
    volatile uint32_t trace_write_index;
    volatile uint32_t trace_count;
    volatile uint32_t trace_wrap_count;
    volatile uint8_t pending_mask;
    volatile uint8_t success_mask;
    volatile uint8_t error_mask;
    volatile uint8_t active_slave;
    volatile uint8_t fail_valid;
    volatile uint8_t fail_slave_num;
    volatile uint8_t fail_phase;
    volatile uint8_t fail_event;
    volatile uint8_t fail_terminal_reason;
    volatile uint8_t fail_detail_reason;
    volatile uint8_t fail_response;
    volatile uint8_t fail_retry_count;
    volatile uint8_t fail_ack;
    volatile uint32_t fail_tick;
    volatile uint32_t fail_last_command_tick;
    volatile uint32_t fail_bytes_written;
    volatile uint32_t fail_total_bytes;
    volatile uint32_t fail_target_address;
    volatile uint16_t fail_payload_size;
    volatile uint16_t fail_padded_size;
    volatile uint16_t fail_erase_next_page;
    volatile uint16_t fail_erase_last_page;
    volatile uint16_t fail_erase_pages_in_block;
    volatile char fail_phase_text[SLAVE_UPDATE_DEBUG_TEXT_SIZE];
    volatile char fail_reason_text[SLAVE_UPDATE_DEBUG_TEXT_SIZE];
    volatile char fail_detail_text[SLAVE_UPDATE_DEBUG_TEXT_SIZE];
} SlaveUpdateDebugInfo_t;

typedef struct
{
    uint32_t sequence;
    uint32_t tick;
    uint32_t bytes_written;
    uint32_t total_bytes;
    uint32_t target_address;
    uint16_t payload_size;
    uint16_t padded_size;
    uint8_t slave_num;
    uint8_t phase;
    uint8_t event;
    uint8_t reason;
    uint8_t response;
    uint8_t retry_count;
} SlaveUpdateTraceEntry_t;

extern SlaveSystem_t slave;
extern volatile SlaveUpdateDebugInfo_t g_slave_update_debug;
extern volatile SlaveUpdateTraceEntry_t g_slave_update_trace[SLAVE_UPDATE_TRACE_DEPTH];

void Slave_UpdateProcess(void);
bool Slave_IsUpdateActive(uint8_t slave_num);
bool Slave_IsAnyUpdateRunning(void);
bool Slave_RequestSingleUpdate(uint8_t slave_num);
bool Slave_RequestConnectedUpdate(void);
bool Slave_RequestAutoConnectedUpdate(void);
void Slave_CancelPendingUpdates(void);
void Slave_OnModbusFrameReceived(uint8_t slave_num, const uint8_t *frame, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_UPDATE_THREAD_H */
