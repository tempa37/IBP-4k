#ifndef SLAVE_UPDATE_THREAD_H
#define SLAVE_UPDATE_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "uart.h"

#include <stdbool.h>
#include <stdint.h>

#define SLAVE_UPDATE_TIMEOUT_MS           15000u
#define SLAVE_UPDATE_ERASE_TIMEOUT_MS     120000u
#define SLAVE_UPDATE_RETRY_MAX            5u
#define SLAVE_FW_BLOCK_SIZE               256u
#define SLAVE_FW_SEND_ENABLE              1u
#define SLAVE_AUTO_UPDATE_ENABLE          1u
#define SLAVE_BOOTLOADER_ENTRY_DELAY_MS   500u
#define SLAVE_FLASH_TARGET_START_ADDR     FLASH_APP_START_ADDR

#define BOOTLOADER_ACK                    0x79u
#define BOOTLOADER_NACK                   0x1Fu
#define BOOTLOADER_SYNC                   0x7Fu
#define BOOTLOADER_EXT_ERASE              0x44u
#define BOOTLOADER_GO                     0x21u
#define BOOTLOADER_WRITE                  0x31u

#define SLAVE_UPDATE_DEBUG_EVENT_NONE     0u
#define SLAVE_UPDATE_DEBUG_EVENT_DONE     1u
#define SLAVE_UPDATE_DEBUG_EVENT_ERROR    2u

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
    volatile uint8_t pending_mask;
    volatile uint8_t success_mask;
    volatile uint8_t error_mask;
    volatile uint8_t active_slave;
} SlaveUpdateDebugInfo_t;

extern SlaveSystem_t slave;
extern volatile SlaveUpdateDebugInfo_t g_slave_update_debug;

void Slave_UpdateProcess(void);
bool Slave_IsUpdateActive(uint8_t slave_num);
bool Slave_IsAnyUpdateRunning(void);
bool Slave_RequestSingleUpdate(uint8_t slave_num);
bool Slave_RequestConnectedUpdate(void);
bool Slave_RequestAutoConnectedUpdate(void);
void Slave_CancelPendingUpdates(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_UPDATE_THREAD_H */
