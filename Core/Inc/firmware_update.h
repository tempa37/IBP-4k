#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define FW_IMAGE_KIND_NONE      0u
#define FW_IMAGE_KIND_MASTER    1u
#define FW_IMAGE_KIND_SLAVE     2u

#define FW_UPDATE_TRACE_DEPTH                32u

#define FW_UPDATE_DEBUG_EVENT_NONE           0u
#define FW_UPDATE_DEBUG_EVENT_BEGIN_RX       1u
#define FW_UPDATE_DEBUG_EVENT_BEGIN_ACCEPT   2u
#define FW_UPDATE_DEBUG_EVENT_BEGIN_REJECT   3u
#define FW_UPDATE_DEBUG_EVENT_BLOCK_WRITTEN  4u
#define FW_UPDATE_DEBUG_EVENT_SESSION_DONE   5u
#define FW_UPDATE_DEBUG_EVENT_METADATA_OK    6u
#define FW_UPDATE_DEBUG_EVENT_METADATA_FAIL  7u
#define FW_UPDATE_DEBUG_EVENT_AUTOSTART_OK   8u
#define FW_UPDATE_DEBUG_EVENT_AUTOSTART_FAIL 9u
#define FW_UPDATE_DEBUG_EVENT_FLASH_ERROR    10u
#define FW_UPDATE_DEBUG_EVENT_START_CMD      11u

#define FW_UPDATE_DEBUG_STAGE_IDLE           0u
#define FW_UPDATE_DEBUG_STAGE_BEGIN          1u
#define FW_UPDATE_DEBUG_STAGE_RX             2u
#define FW_UPDATE_DEBUG_STAGE_STORED         3u
#define FW_UPDATE_DEBUG_STAGE_SLAVE_START    4u
#define FW_UPDATE_DEBUG_STAGE_DONE           5u
#define FW_UPDATE_DEBUG_STAGE_ERROR          6u

#define FW_UPDATE_DEBUG_REASON_NONE               0u
#define FW_UPDATE_DEBUG_REASON_BUSY               1u
#define FW_UPDATE_DEBUG_REASON_BAD_BEGIN          2u
#define FW_UPDATE_DEBUG_REASON_BAD_IMAGE          3u
#define FW_UPDATE_DEBUG_REASON_ERASE_FAILED       4u
#define FW_UPDATE_DEBUG_REASON_METADATA_FAILED    5u
#define FW_UPDATE_DEBUG_REASON_FLASH_WRITE_FAILED 6u
#define FW_UPDATE_DEBUG_REASON_BAD_DATA           7u
#define FW_UPDATE_DEBUG_REASON_CRC_FAILED         8u
#define FW_UPDATE_DEBUG_REASON_AUTO_REQUEST_FAILED 9u

typedef struct {
    uint32_t sequence;
    uint32_t tick;
    uint32_t imageSizeBytes;
    uint32_t bytesWritten;
    uint32_t writeAddress;
    uint16_t totalBlocks;
    uint16_t currentBlock;
    uint16_t packetIndex;
    uint16_t msgId;
    uint8_t event;
    uint8_t stage;
    uint8_t reason;
    uint8_t imageKind;
    uint8_t srcNode;
    uint8_t dataLength;
    uint8_t packetInBlock;
} FirmwareUpdateTraceEntry_t;

typedef struct {
    volatile uint32_t eventCounter;
    volatile uint32_t traceWriteIndex;
    volatile uint32_t traceCount;
    volatile uint32_t traceWrapCount;
    volatile uint32_t lastTick;
    volatile uint32_t lastImageSizeBytes;
    volatile uint32_t lastBytesWritten;
    volatile uint32_t lastWriteAddress;
    volatile uint16_t lastTotalBlocks;
    volatile uint16_t lastBlockNum;
    volatile uint16_t lastPacketIndex;
    volatile uint16_t lastMsgId;
    volatile uint8_t lastEvent;
    volatile uint8_t lastStage;
    volatile uint8_t lastReason;
    volatile uint8_t lastImageKind;
    volatile uint8_t lastSrcNode;
    volatile uint8_t lastDataLength;
    volatile uint8_t lastPacketInBlock;
    volatile uint8_t sessionActive;
} FirmwareUpdateDebugInfo_t;

extern volatile FirmwareUpdateDebugInfo_t g_fw_update_debug;
extern volatile FirmwareUpdateTraceEntry_t g_fw_update_trace[FW_UPDATE_TRACE_DEPTH];
extern volatile uint32_t storedImageType;
extern volatile uint32_t storedImageSize;
extern volatile uint32_t storedUpdateFlag;

/* Загружает из flash сведения о принятом firmware-образе. */
void FW_LoadStoredImageInfo(void);

/* Обрабатывает входящую extended CAN-команду протокола обновления прошивки. */
bool FW_HandleExtendedUpdateCommand(uint32_t extId, const uint8_t *canData, uint8_t dataLength);

/* Фиксирует готовность master-образа к применению после перезапуска. */
HAL_StatusTypeDef Set_Update_Flag(uint32_t imageSizeBytes);

#ifdef __cplusplus
}
#endif

#endif /* FIRMWARE_UPDATE_H */
