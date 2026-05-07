#include "firmware_update.h"

#include "app_diagnostics.h"
#include "can.h"
#include "flash_storage.h"
#include "slave_update_thread.h"
#include "stm32f4xx_hal_flash.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/*
 * RAM-состояние одного сеанса приёма прошивки по CAN.
 *
 *
 * Принцип:
 * - на BEGIN структура инициализируется;
 * - на DATA блок накапливается в blockBuffer;
 * - после завершения данные о сохранённом образе переносятся в metadata во flash.
 */
typedef struct {
    uint8_t imageKind;                 // Тип образа: master/slave
    bool updateInProgress;              // Флаг: идет процесс обновления
    bool imageSizeExact;               // Размер передан точно, а не выведен из количества блоков
    uint16_t totalBlocksExpected;       // Общее количество блоков к записи
    uint16_t currentBlockNum;           // Номер текущего блока (0, 1, 2, ...)
    uint8_t packetsInCurrentBlock;      // Сколько пакетов уже получено в текущем блоке (0..9)
    uint8_t blockBuffer[FW_BLOCK_SIZE_BYTES]; // Буфер для накопления одного блока
    uint32_t writeAddress;              // Текущий адрес записи во flash
    uint32_t imageSizeBytes;           // Фактический размер принятого образа
    uint32_t bytesWritten;             // Сколько байт образа уже записано во flash
} FirmwareUpdateState_t;

/*
 * Два шаблона состояния нужны только для разделения логики master/slave.
 * Физическая flash-область при этом одна общая, а активен в каждый момент
 * только один сеанс, на который указывает activeFwState.
 */
static FirmwareUpdateState_t slaveFwState = {0};
static FirmwareUpdateState_t masterFwState = {0};
static FirmwareUpdateState_t *activeFwState = NULL;



volatile FirmwareUpdateDebugInfo_t g_fw_update_debug = {0};
volatile FirmwareUpdateTraceEntry_t g_fw_update_trace[FW_UPDATE_TRACE_DEPTH] = {0};


volatile uint32_t storedImageType = FW_IMAGE_KIND_NONE;
volatile uint32_t storedImageSize = 0u;
volatile uint32_t storedUpdateFlag = FLASH_UPDATE_FLAG_CLEAR_VALUE;

static void FW_HandleDataPacket(FirmwareUpdateState_t *state,
                                uint8_t dstNode,
                                uint16_t msgId,
                                const uint8_t *canData,
                                uint8_t dataLength);
static void FW_SendAck(uint8_t dstNode, const uint8_t *data, uint8_t dlc);
static bool FW_ParseBeginRequest(const uint8_t *canData,
                                 uint8_t dataLength,
                                 uint16_t *totalBlocks,
                                 uint32_t *imageSizeBytes,
                                 bool *imageSizeExact);
static bool FW_IsStoredImageInfoValid(uint8_t imageKind, uint32_t imageSizeBytes);
static uint16_t FW_CalculateExpectedBlocks(uint32_t imageSizeBytes);
static uint32_t FW_NormalizeImageSize(uint32_t imageSizeBytes);
static FirmwareUpdateState_t *FW_SelectStateByImageSize(uint32_t imageSizeBytes);
static uint32_t FW_GetStorageStartAddress(const FirmwareUpdateState_t *state);
static HAL_StatusTypeDef FW_EraseStorageForState(FirmwareUpdateState_t *state);
static uint32_t FW_EncodeStoredImageType(uint8_t imageKind);
static uint8_t FW_DecodeStoredImageType(uint32_t rawType);
static void FW_ApplyStoredImageInfo(uint8_t imageKind, uint32_t imageSizeBytes, uint32_t updateFlagValue);
static HAL_StatusTypeDef FW_WriteStoredImageInfo(uint8_t imageKind,
                                                 uint32_t imageSizeBytes,
                                                 uint32_t updateFlagValue);
static bool FW_VerifyStoredImageCrc(const FirmwareUpdateState_t *state,
                                    uint32_t *calculatedCrc,
                                    uint32_t *storedCrc);
static uint8_t FW_GetImageKindByState(const FirmwareUpdateState_t *state);
static void FW_RecordDebugEvent(const FirmwareUpdateState_t *state,
                                uint8_t srcNode,
                                uint16_t msgId,
                                uint8_t dataLength,
                                uint16_t packetIndex,
                                uint8_t packetInBlock,
                                uint8_t event,
                                uint8_t stage,
                                uint8_t reason);

/*
 * Разбор BEGIN-пакета обновления.
 *
 * Что делает:
 * - достаёт число блоков;
 * - если отправитель передал точный размер, забирает и его;
 * - если точного размера нет, грубо восстанавливает размер как blocks * 60.
 *
 * Почему это нужно:
 * - вся дальнейшая логика выбора образа и проверки ожидаемого числа блоков
 *   завязана именно на размере.
 */
static bool FW_ParseBeginRequest(const uint8_t *canData,
                                 uint8_t dataLength,
                                 uint16_t *totalBlocks,
                                 uint32_t *imageSizeBytes,
                                 bool *imageSizeExact)
{
    uint16_t parsedBlocks = 0u;
    uint32_t parsedSize = 0u;
    bool parsedSizeIsExact = false;

    if ((canData == NULL) || (totalBlocks == NULL) || (imageSizeBytes == NULL) || (imageSizeExact == NULL))
    {
        return false;
    }

    if (dataLength < 2u)
    {
        return false;
    }

    parsedBlocks = (uint16_t)((canData[0] << 8) | canData[1]);

    if (dataLength >= 6u)
    {
        parsedSize = ((uint32_t)canData[2] << 24)
                   | ((uint32_t)canData[3] << 16)
                   | ((uint32_t)canData[4] << 8)
                   | (uint32_t)canData[5];
        parsedSizeIsExact = true;
    }
    else
    {
        parsedSize = (uint32_t)parsedBlocks * FW_BLOCK_SIZE_BYTES;
    }

    *totalBlocks = parsedBlocks;
    *imageSizeBytes = FW_NormalizeImageSize(parsedSize);
    *imageSizeExact = parsedSizeIsExact;

    return true;
}

/*
 * Проверка согласованности metadata.
 *
 * Здесь не происходит определения типа образа "с нуля". Функция только отвечает
 * на вопрос: можно ли доверять комбинации type + size, 
 * прочитанной из flash, или же там явно мусор
 *
 */
static bool FW_IsStoredImageInfoValid(uint8_t imageKind, uint32_t imageSizeBytes)
{
    if (imageKind == FW_IMAGE_KIND_NONE)
    {
        return (imageSizeBytes == 0u);
    }

    if (imageKind == FW_IMAGE_KIND_MASTER)
    {
        return (imageSizeBytes == FLASH_APP_SIZE);
    }

    if (imageKind == FW_IMAGE_KIND_SLAVE)
    {
        return ((imageSizeBytes >= sizeof(uint32_t)) &&
                (imageSizeBytes < FLASH_APP_SIZE) &&
                (imageSizeBytes <= FLASH_FW_STORAGE_SIZE) &&
                (imageSizeBytes <= SLAVE_FLASH_TARGET_SIZE_BYTES));
    }

    return false;
}

/*
 * Перевод размера образа в ожидаемое число transport-блоков по 60 байт.
 *
 * Нужен как sanity-check: BEGIN не должен врать сам себе. Если отправитель
 * заявил размер и число блоков, а математика не сходится, приём не запускаем.
 */
static uint16_t FW_CalculateExpectedBlocks(uint32_t imageSizeBytes)
{
    if (imageSizeBytes == 0u)
    {
        return 0u;
    }

    return (uint16_t)((imageSizeBytes + FW_BLOCK_SIZE_BYTES - 1u) / FW_BLOCK_SIZE_BYTES);
}

/*
 * Нормализация размера образа.
 *
 * Зачем нужна:
 * - последний блок протокола всегда добивается паддингом до 60 байт;
 * - master firmware имеет жёсткий логический размер 64 KB;
 * - если отправитель прислал 64 KB + хвост паддинга в пределах одного блока,
 *   это всё ещё надо считать master firmware.
 */
static uint32_t FW_NormalizeImageSize(uint32_t imageSizeBytes)
{
    if ((imageSizeBytes >= FLASH_APP_SIZE) &&
        (imageSizeBytes <= (FLASH_APP_SIZE + FW_BLOCK_SIZE_BYTES - 1u)))
    {
        return FLASH_APP_SIZE;
    }

    return imageSizeBytes;
}

/*
 * Кодирование внутреннего enum типа образа в flash-сигнатуру.
 *
 * Во flash храним не просто 1/2, а читаемые сигнатуры MAST / SLAV..
 */
static uint32_t FW_EncodeStoredImageType(uint8_t imageKind)
{
    if (imageKind == FW_IMAGE_KIND_MASTER)
    {
        return FLASH_STORED_IMAGE_TYPE_MASTER_VALUE;
    }

    if (imageKind == FW_IMAGE_KIND_SLAVE)
    {
        return FLASH_STORED_IMAGE_TYPE_SLAVE_VALUE;
    }

    return FLASH_STORED_IMAGE_TYPE_NONE_VALUE;
}

/*
 * Обратное преобразование flash-сигнатуры в enum образа.
 *
 * Всё, что не похоже на известные MAST / SLAV, считаем NONE. 
 */
static uint8_t FW_DecodeStoredImageType(uint32_t rawType)
{
    if (rawType == FLASH_STORED_IMAGE_TYPE_MASTER_VALUE)
    {
        return FW_IMAGE_KIND_MASTER;
    }

    if (rawType == FLASH_STORED_IMAGE_TYPE_SLAVE_VALUE)
    {
        return FW_IMAGE_KIND_SLAVE;
    }

    return FW_IMAGE_KIND_NONE;
}

/*
 * Применение metadata к RAM-состоянию.
 *
 * Здесь нет записи во flash. Это только синхронизация оперативных переменных
 * после чтения metadata или после успешной записи metadata.
 *
 * Отдельно важно:
 * - если сохранён slave image, автоматически выставляются slave.os_in_flash
 *   и slave.os_size_bytes;
 * - если сохранён master image или ничего не сохранено, slave-флаги обнуляются.
 */
static void FW_ApplyStoredImageInfo(uint8_t imageKind,
                                    uint32_t imageSizeBytes,
                                    uint32_t updateFlagValue)
{
    storedImageType = imageKind;
    storedImageSize = imageSizeBytes;
    storedUpdateFlag = (updateFlagValue == FLASH_UPDATE_FLAG_SET_VALUE)
                     ? FLASH_UPDATE_FLAG_SET_VALUE
                     : FLASH_UPDATE_FLAG_CLEAR_VALUE;

    if ((imageKind == FW_IMAGE_KIND_SLAVE) && (imageSizeBytes > 0u))
    {
        slave.os_in_flash = 1u;
        slave.os_size_bytes = imageSizeBytes;
    }
    else
    {
        slave.os_in_flash = 0u;
        slave.os_size_bytes = 0u;
    }
}

/*
 * Загрузка metadata из flash на старте приложения.
 *
 * Принцип:
 * 1. читаем сырые слова из сектора metadata;
 * 2. декодируем тип;
 * 3. проверяем, что комбинация type + size валидна;
 * 4. переносим результат в RAM-переменные.
 *
 * Так приложение после reset понимает, что именно сейчас лежит в общем
 * staging-слоте
 */
void FW_LoadStoredImageInfo(void)
{
    uint32_t rawUpdateFlag = *(const uint32_t *)FLASH_UPDATE_FLAG_ADDR;
    uint32_t rawImageType = *(const uint32_t *)FLASH_STORED_IMAGE_TYPE_ADDR;
    uint32_t rawImageSize = *(const uint32_t *)FLASH_STORED_IMAGE_SIZE_ADDR;
    uint8_t imageKind = FW_DecodeStoredImageType(rawImageType);
    uint32_t imageSizeBytes = rawImageSize;

    if (!FW_IsStoredImageInfoValid(imageKind, imageSizeBytes))
    {
        imageKind = FW_IMAGE_KIND_NONE;
        imageSizeBytes = 0u;
    }

    FW_ApplyStoredImageInfo(imageKind, imageSizeBytes, rawUpdateFlag);
}

/*
 * Полная перезапись metadata во flash.
 *
 * Почему функция стирает весь сектор:
 * - metadata и update-flag живут в одном секторе;
 * - поэтому запись любого из этих полей всегда считается атомарным обновлением
 *   всей тройки: update flag + image type + image size.
 *
 * Что важно понимать:
 * - новый приём образа сначала очищает metadata до NONE;
 * - metadata любого образа пишется только после успешной CRC-проверки;
 * - сохранение slave image пишет тип и размер, но оставляет update-flag пустым;
 * - сохранение master image пишет и тип, и размер, и update-flag.
 */
static HAL_StatusTypeDef FW_WriteStoredImageInfo(uint8_t imageKind,
                                                 uint32_t imageSizeBytes,
                                                 uint32_t updateFlagValue)
{
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0u;
    uint32_t rawImageType = FW_EncodeStoredImageType(imageKind);

    if (!FW_IsStoredImageInfoValid(imageKind, imageSizeBytes))
    {
        return HAL_ERROR;
    }

    if ((updateFlagValue != FLASH_UPDATE_FLAG_SET_VALUE) &&
        (updateFlagValue != FLASH_UPDATE_FLAG_CLEAR_VALUE))
    {
        return HAL_ERROR;
    }

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = FLASH_UPDATE_FLAG_SECTOR;
    eraseInit.NbSectors = 1u;

    HAL_FLASH_Unlock();

    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    if (status == HAL_OK)
    {
        if (updateFlagValue != FLASH_UPDATE_FLAG_CLEAR_VALUE)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_UPDATE_FLAG_ADDR,
                                       updateFlagValue);
        }

        if ((status == HAL_OK) && (rawImageType != FLASH_STORED_IMAGE_TYPE_NONE_VALUE))
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_STORED_IMAGE_TYPE_ADDR,
                                       rawImageType);
        }

        if ((status == HAL_OK) && (imageSizeBytes != 0u))
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_STORED_IMAGE_SIZE_ADDR,
                                       imageSizeBytes);
        }
    }

    HAL_FLASH_Lock();

    if (status == HAL_OK)
    {
        FW_ApplyStoredImageInfo(imageKind, imageSizeBytes, updateFlagValue);
    }

    return status;
}

/*
 * Выбор логического типа принимаемого образа.
 *
 */
static FirmwareUpdateState_t *FW_SelectStateByImageSize(uint32_t imageSizeBytes)
{
    if (imageSizeBytes == FLASH_APP_SIZE)
    {
        return &masterFwState;
    }

    if ((imageSizeBytes >= sizeof(uint32_t)) &&
        (imageSizeBytes < FLASH_APP_SIZE) &&
        (imageSizeBytes <= FLASH_FW_STORAGE_SIZE) &&
        (imageSizeBytes <= SLAVE_FLASH_TARGET_SIZE_BYTES))
    {
        return &slaveFwState;
    }

    return NULL;
}

/*
 * Возврат физического адреса начала staging-слота (он один).
 */
static uint32_t FW_GetStorageStartAddress(const FirmwareUpdateState_t *state)
{
    if ((state == &masterFwState) || (state == &slaveFwState))
    {
        return FLASH_FW_STORAGE_START_ADDR;
    }

    return 0u;
}

/*
 * Стирание общей staging-области.
 */
static HAL_StatusTypeDef FW_EraseStorageForState(FirmwareUpdateState_t *state)
{
    if ((state == &masterFwState) || (state == &slaveFwState))
    {
        HAL_StatusTypeDef eraseStatus;

        taskENTER_CRITICAL();
        eraseStatus = FLASH_EraseSectors(FLASH_FW_STORAGE_START_SECTOR, FLASH_FW_STORAGE_END_SECTOR);
        taskEXIT_CRITICAL();
        return eraseStatus;
    }

    return HAL_ERROR;
}

/*
 * Проверка CRC сохранённого образа.
 *
 * Функция используется для master и slave firmware. Оба формата образа должны
 * хранить эталонный CRC32 в последних 4 байтах, иначе metadata не записывается
 * и дальнейшее обновление не запускается. 
 *
 * Принцип:
 * - считаем CRC по образу без последних 4 байт;
 * - последние 4 байта считаем сохранённым эталонным CRC;
 * - сравниваем значения и возвращаем результат.
 */
static bool FW_VerifyStoredImageCrc(const FirmwareUpdateState_t *state,
                                    uint32_t *calculatedCrc,
                                    uint32_t *storedCrc)
{
    uint32_t imageStart = 0u;
    uint32_t payloadLength = 0u;
    const uint8_t *crcPtr;

    if ((state == NULL) || (state->imageSizeBytes < sizeof(uint32_t)) ||
        (calculatedCrc == NULL) || (storedCrc == NULL))
    {
        return false;
    }

    imageStart = FW_GetStorageStartAddress(state);
    if (imageStart == 0u)
    {
        return false;
    }

    payloadLength = state->imageSizeBytes - sizeof(uint32_t);
    *calculatedCrc = compute_flash_crc(imageStart, payloadLength);

    crcPtr = (const uint8_t *)(imageStart + payloadLength);
    *storedCrc = ((uint32_t)crcPtr[0])
               | ((uint32_t)crcPtr[1] << 8)
               | ((uint32_t)crcPtr[2] << 16)
               | ((uint32_t)crcPtr[3] << 24);

    return (*calculatedCrc == *storedCrc);
}

/* Определяет тип образа по адресу активного RAM-состояния приема. */
static uint8_t FW_GetImageKindByState(const FirmwareUpdateState_t *state)
{
    if (state == &masterFwState)
    {
        return FW_IMAGE_KIND_MASTER;
    }

    if (state == &slaveFwState)
    {
        return FW_IMAGE_KIND_SLAVE;
    }

    return FW_IMAGE_KIND_NONE;
}

/* Сохраняет одно событие приема прошивки в отладочный снимок и trace. */
static void FW_RecordDebugEvent(const FirmwareUpdateState_t *state,
                                uint8_t srcNode,
                                uint16_t msgId,
                                uint8_t dataLength,
                                uint16_t packetIndex,
                                uint8_t packetInBlock,
                                uint8_t event,
                                uint8_t stage,
                                uint8_t reason)
{
    uint32_t tick = HAL_GetTick();
    uint32_t writeIndex = g_fw_update_debug.traceWriteIndex;
    uint32_t nextIndex = (writeIndex + 1u) % FW_UPDATE_TRACE_DEPTH;
    uint32_t sequence = g_fw_update_debug.eventCounter + 1u;
    uint8_t imageKind = FW_GetImageKindByState(state);
    volatile FirmwareUpdateTraceEntry_t *entry = &g_fw_update_trace[writeIndex];

    g_fw_update_debug.eventCounter = sequence;
    g_fw_update_debug.traceWriteIndex = nextIndex;
    if (g_fw_update_debug.traceCount < FW_UPDATE_TRACE_DEPTH)
    {
        g_fw_update_debug.traceCount++;
    }
    else
    {
        g_fw_update_debug.traceWrapCount++;
    }

    g_fw_update_debug.lastTick = tick;
    g_fw_update_debug.lastEvent = event;
    g_fw_update_debug.lastStage = stage;
    g_fw_update_debug.lastReason = reason;
    g_fw_update_debug.lastImageKind = imageKind;
    g_fw_update_debug.lastSrcNode = srcNode;
    g_fw_update_debug.lastMsgId = msgId;
    g_fw_update_debug.lastDataLength = dataLength;
    g_fw_update_debug.lastPacketIndex = packetIndex;
    g_fw_update_debug.lastPacketInBlock = packetInBlock;
    g_fw_update_debug.lastImageSizeBytes = (state != NULL) ? state->imageSizeBytes : 0u;
    g_fw_update_debug.lastBytesWritten = (state != NULL) ? state->bytesWritten : 0u;
    g_fw_update_debug.lastWriteAddress = (state != NULL) ? state->writeAddress : 0u;
    g_fw_update_debug.lastTotalBlocks = (state != NULL) ? state->totalBlocksExpected : 0u;
    g_fw_update_debug.lastBlockNum = (state != NULL) ? state->currentBlockNum : 0u;
    g_fw_update_debug.sessionActive = ((state != NULL) && state->updateInProgress) ? 1u : 0u;

    entry->sequence = sequence;
    entry->tick = tick;
    entry->imageSizeBytes = g_fw_update_debug.lastImageSizeBytes;
    entry->bytesWritten = g_fw_update_debug.lastBytesWritten;
    entry->writeAddress = g_fw_update_debug.lastWriteAddress;
    entry->totalBlocks = g_fw_update_debug.lastTotalBlocks;
    entry->currentBlock = g_fw_update_debug.lastBlockNum;
    entry->packetIndex = packetIndex;
    entry->msgId = msgId;
    entry->event = event;
    entry->stage = stage;
    entry->reason = reason;
    entry->imageKind = imageKind;
    entry->srcNode = srcNode;
    entry->dataLength = dataLength;
    entry->packetInBlock = packetInBlock;
}

/*
 * Отправка ACK по CAN для протокола обновления.
 *
 * ACK используется в двух местах:
 * - после BEGIN приёмник сообщает состояние стирания/готовности;
 * - после записи блока приёмник подтверждает номер записанного блока.
 *
 */
static void FW_SendAck(uint8_t dstNode, const uint8_t *data, uint8_t dlc)
{
    uint32_t ackId = CAN_BuildExtId(CAN_NODE_IBP_4K,
                                    dstNode,
                                    CAN_MSG_FW_ACK,
                                    CAN_PRIORITY_DEFAULT);

    CAN_SendExtendedFrame(&hcan1, ackId, data, dlc);
}

/* Обрабатывает CAN-команды протокола приема firmware-образа. */
bool FW_HandleExtendedUpdateCommand(uint32_t extId, const uint8_t *canData, uint8_t dataLength)
{
    uint8_t srcNode = 0u;
    uint8_t dstNode = 0u;
    uint16_t msgId = 0u;

    CAN_ParseExtId(extId, &srcNode, &dstNode, &msgId, NULL);

    if ((srcNode != CAN_NODE_KOU) || (dstNode != CAN_NODE_IBP_4K))
    {
        return false;
    }

    switch (msgId)
    {
        case CAN_MSG_FW_SLAVE_BEGIN:
        case CAN_MSG_FW_MASTER_BEGIN:
        {
            /*
             * BEGIN master/slave обрабатываются одинаково.
             *
             * Принцип:
             * 1. читаем размер и число блоков;
             * 2. по размеру определяем, master это или slave;
             * 3. стираем общий staging-слот;
             * 4. очищаем metadata до NONE;
             * 5. запускаем новый активный сеанс приёма.
             */
            uint16_t totalBlocks = 0u;
            uint32_t imageSizeBytes = 0u;
            bool imageSizeExact = false;
            uint8_t responseData[1] = {0u};
            HAL_StatusTypeDef eraseStatus = HAL_ERROR;
            HAL_StatusTypeDef metadataStatus = HAL_ERROR;
            FirmwareUpdateState_t *selectedState = NULL;
            bool blocksMatch = false;

            FW_RecordDebugEvent(NULL,
                                srcNode,
                                msgId,
                                dataLength,
                                0u,
                                0u,
                                FW_UPDATE_DEBUG_EVENT_BEGIN_RX,
                                FW_UPDATE_DEBUG_STAGE_BEGIN,
                                FW_UPDATE_DEBUG_REASON_NONE);

            if (Slave_IsAnyUpdateRunning())
            {
                FW_RecordDebugEvent(NULL,
                                    srcNode,
                                    msgId,
                                    dataLength,
                                    0u,
                                    0u,
                                    FW_UPDATE_DEBUG_EVENT_BEGIN_REJECT,
                                    FW_UPDATE_DEBUG_STAGE_ERROR,
                                    FW_UPDATE_DEBUG_REASON_BUSY);
                FW_SendAck(srcNode, responseData, 1u);
                return true;
            }

            if (!FW_ParseBeginRequest(canData,
                                      dataLength,
                                      &totalBlocks,
                                      &imageSizeBytes,
                                      &imageSizeExact))
            {
                FW_RecordDebugEvent(NULL,
                                    srcNode,
                                    msgId,
                                    dataLength,
                                    0u,
                                    0u,
                                    FW_UPDATE_DEBUG_EVENT_BEGIN_REJECT,
                                    FW_UPDATE_DEBUG_STAGE_ERROR,
                                    FW_UPDATE_DEBUG_REASON_BAD_BEGIN);
                FW_SendAck(srcNode, responseData, 1u);
                return true;
            }

            selectedState = FW_SelectStateByImageSize(imageSizeBytes);
            blocksMatch = (totalBlocks == FW_CalculateExpectedBlocks(imageSizeBytes));

            if ((selectedState != NULL) && blocksMatch)
            {
                eraseStatus = FW_EraseStorageForState(selectedState);
                if (eraseStatus == HAL_OK)
                {
                    metadataStatus = FW_WriteStoredImageInfo(FW_IMAGE_KIND_NONE,
                                                             0u,
                                                             FLASH_UPDATE_FLAG_CLEAR_VALUE);
                }
            }

            if ((selectedState != NULL) &&
                (eraseStatus == HAL_OK) &&
                (metadataStatus == HAL_OK) &&
                blocksMatch)
            {
                memset(&slaveFwState, 0, sizeof(slaveFwState));
                memset(&masterFwState, 0, sizeof(masterFwState));

                selectedState->imageKind = (selectedState == &masterFwState)
                                         ? FW_IMAGE_KIND_MASTER
                                         : FW_IMAGE_KIND_SLAVE;
                selectedState->updateInProgress = true;
                selectedState->imageSizeExact = imageSizeExact;
                selectedState->totalBlocksExpected = totalBlocks;
                selectedState->currentBlockNum = 0u;
                selectedState->packetsInCurrentBlock = 0u;
                selectedState->writeAddress = FW_GetStorageStartAddress(selectedState);
                selectedState->imageSizeBytes = imageSizeBytes;
                selectedState->bytesWritten = 0u;
                memset(selectedState->blockBuffer, 0xFF, sizeof(selectedState->blockBuffer));

                Slave_CancelPendingUpdates();
                activeFwState = selectedState;
                responseData[0] = 0xFFu;
                FW_RecordDebugEvent(selectedState,
                                    srcNode,
                                    msgId,
                                    dataLength,
                                    0u,
                                    0u,
                                    FW_UPDATE_DEBUG_EVENT_BEGIN_ACCEPT,
                                    FW_UPDATE_DEBUG_STAGE_BEGIN,
                                    FW_UPDATE_DEBUG_REASON_NONE);
            }
            else
            {
                activeFwState = NULL;
                if ((selectedState == NULL) || !blocksMatch)
                {
                    FW_RecordDebugEvent(NULL,
                                        srcNode,
                                        msgId,
                                        dataLength,
                                        0u,
                                        0u,
                                        FW_UPDATE_DEBUG_EVENT_BEGIN_REJECT,
                                        FW_UPDATE_DEBUG_STAGE_ERROR,
                                        FW_UPDATE_DEBUG_REASON_BAD_IMAGE);
                }
                else if (eraseStatus != HAL_OK)
                {
                    FW_RecordDebugEvent(selectedState,
                                        srcNode,
                                        msgId,
                                        dataLength,
                                        0u,
                                        0u,
                                        FW_UPDATE_DEBUG_EVENT_FLASH_ERROR,
                                        FW_UPDATE_DEBUG_STAGE_ERROR,
                                        FW_UPDATE_DEBUG_REASON_ERASE_FAILED);
                }
                else if (metadataStatus != HAL_OK)
                {
                    FW_RecordDebugEvent(selectedState,
                                        srcNode,
                                        msgId,
                                        dataLength,
                                        0u,
                                        0u,
                                        FW_UPDATE_DEBUG_EVENT_METADATA_FAIL,
                                        FW_UPDATE_DEBUG_STAGE_ERROR,
                                        FW_UPDATE_DEBUG_REASON_METADATA_FAILED);
                }

                if ((selectedState != NULL) &&
                    ((eraseStatus != HAL_OK) || (metadataStatus != HAL_OK)))
                {
                    CAN_ReportFlashWriteError();
                }
            }

            FW_SendAck(srcNode, responseData, 1u);
            return true;
        }

        case CAN_MSG_FW_SLAVE_DATA:
        case CAN_MSG_FW_MASTER_DATA:
            /*
             * DATA для master и slave тоже идут в один обработчик.
             * Источник истины здесь activeFwState, который был
             * выбран и инициализирован на этапе BEGIN.
             */
            FW_HandleDataPacket(activeFwState, srcNode, msgId, canData, dataLength);
            return true;

        case CAN_MSG_FW_SLAVE_UPDATE_START:
        {
            uint8_t slaveNum = 0xFFu;
            uint8_t responseData[2];
            bool requestAccepted = false;

            if (dataLength >= 1u)
            {
                slaveNum = canData[0];
            }

            if (SLAVE_FW_SEND_ENABLE != 0u)
            {
                requestAccepted = (slaveNum < UART_CHANNEL_COUNT)
                                ? Slave_RequestSingleUpdate(slaveNum)
                                : Slave_RequestConnectedUpdate();
            }

            responseData[0] = slaveNum;
            responseData[1] = requestAccepted ? 0xFFu : 0x00u;

            FW_RecordDebugEvent(&slaveFwState,
                                srcNode,
                                msgId,
                                dataLength,
                                0u,
                                0u,
                                FW_UPDATE_DEBUG_EVENT_START_CMD,
                                FW_UPDATE_DEBUG_STAGE_SLAVE_START,
                                requestAccepted
                                    ? FW_UPDATE_DEBUG_REASON_NONE
                                    : FW_UPDATE_DEBUG_REASON_AUTO_REQUEST_FAILED);

            FW_SendAck(srcNode, responseData, 2u);
            return true;
        }

        default:
            return false;
    }
}

/*
 * Приём и запись payload-блоков прошивки.
 *
 * Это центральная транспортная функция обновления.
 *
 * Что она делает:
 * - принимает один CAN-пакет;
 * - кладёт его 6 байт в нужное место blockBuffer;
 * - когда блок собран, пишет его в staging flash;
 * - на последнем блоке завершает сеанс.
 *
 * Что важно по принципу:
 * - блоки считаются логическими кусками по 60 байт;
 * - физически master и slave пишутся в одну staging-область;
 * - оба типа образа проходят CRC-проверку на этапе завершения;
 * - различие после успешной CRC только в metadata:
 *   master ставит update-flag для bootloader,
 *   slave фиксирует сохранённый образ и может запускать рассылку ведомым.
 */
static void FW_HandleDataPacket(FirmwareUpdateState_t *state,
                                uint8_t dstNode,
                                uint16_t msgId,
                                const uint8_t *canData,
                                uint8_t dataLength)
{
    // Проверки корректности
    if (!state || !canData || dataLength < 3u)
    {
        FW_RecordDebugEvent(state,
                            dstNode,
                            msgId,
                            dataLength,
                            0u,
                            0u,
                            FW_UPDATE_DEBUG_EVENT_FLASH_ERROR,
                            FW_UPDATE_DEBUG_STAGE_ERROR,
                            FW_UPDATE_DEBUG_REASON_BAD_DATA);
        return;  // Некорректные данные
    }

    if (!state->updateInProgress)
    {
        FW_RecordDebugEvent(state,
                            dstNode,
                            msgId,
                            dataLength,
                            0u,
                            0u,
                            FW_UPDATE_DEBUG_EVENT_FLASH_ERROR,
                            FW_UPDATE_DEBUG_STAGE_ERROR,
                            FW_UPDATE_DEBUG_REASON_BAD_DATA);
        return;  // Обновление не инициализировано
    }

    // Извлекаем глобальный индекс пакета (первые 2 байта)
    uint16_t globalPacketIndex = (uint16_t)((canData[0] << 8) | canData[1]);

    // Данные прошивки начинаются с 3-го байта
    const uint8_t *payload = &canData[2];
    uint8_t payloadLen = (dataLength >= 2u) ? (dataLength - 2u) : 0u;

    // Ограничиваем размер данных
    if (payloadLen > FW_PACKET_DATA_SIZE)
    {
        payloadLen = FW_PACKET_DATA_SIZE;
    }

    // Вычисляем номер блока и позицию пакета в блоке из глобального индекса
    uint16_t calculatedBlockNum = globalPacketIndex / FW_BLOCK_SIZE_PACKETS;
    uint8_t packetNumInBlock = globalPacketIndex % FW_BLOCK_SIZE_PACKETS;


    // Если пришел пакет не из текущего блока - начинаем новый блок
    if (calculatedBlockNum != state->currentBlockNum)
    {
        // Сбрасываем текущий блок и переходим к новому
        state->currentBlockNum = calculatedBlockNum;
        state->packetsInCurrentBlock = 0u;
        memset(state->blockBuffer, 0xFF, sizeof(state->blockBuffer));  // Заполняем 0xFF на случай неполного блока
    }

    // Копируем данные пакета в буфер блока
    uint8_t offsetInBuffer = packetNumInBlock * FW_PACKET_DATA_SIZE;
    if ((offsetInBuffer + payloadLen) <= FW_BLOCK_SIZE_BYTES)
    {
        memcpy(&state->blockBuffer[offsetInBuffer], payload, payloadLen);
    }

    // Обновляем счетчик принятых пакетов
    if (packetNumInBlock >= state->packetsInCurrentBlock)
    {
        state->packetsInCurrentBlock = packetNumInBlock + 1u;
    }

    // Проверяем, собран ли блок полностью
    bool blockComplete = (state->packetsInCurrentBlock >= FW_BLOCK_SIZE_PACKETS);

    // Проверяем, это последний блок?

    // Если блок собран полностью ИЛИ это последний блок и пришли все пакеты
    if (blockComplete)
    {
        // Вычисляем количество байт для записи
        uint32_t remainingBytes = 0u;
        uint32_t bytesToWrite = 0u;
        uint8_t ackData[2];
        uint32_t calculatedCrc = 0u;
        uint32_t storedCrc = 0u;
        HAL_StatusTypeDef writeStatus;

        if (state->bytesWritten >= state->imageSizeBytes)
        {
            state->updateInProgress = false;
            activeFwState = NULL;
            FW_RecordDebugEvent(state,
                                dstNode,
                                msgId,
                                dataLength,
                                globalPacketIndex,
                                packetNumInBlock,
                                FW_UPDATE_DEBUG_EVENT_FLASH_ERROR,
                                FW_UPDATE_DEBUG_STAGE_ERROR,
                                FW_UPDATE_DEBUG_REASON_BAD_DATA);
            return;
        }

        remainingBytes = state->imageSizeBytes - state->bytesWritten;
        bytesToWrite = (remainingBytes > FW_BLOCK_SIZE_BYTES) ? FW_BLOCK_SIZE_BYTES : remainingBytes;

        // Записываем блок во flash
        writeStatus = FLASH_WriteBuffer(state->writeAddress, 
                                        state->blockBuffer, 
                                        bytesToWrite);

        if (writeStatus != HAL_OK)
        {
            state->updateInProgress = false;
            activeFwState = NULL;
            FW_RecordDebugEvent(state,
                                dstNode,
                                msgId,
                                dataLength,
                                globalPacketIndex,
                                packetNumInBlock,
                                FW_UPDATE_DEBUG_EVENT_FLASH_ERROR,
                                FW_UPDATE_DEBUG_STAGE_ERROR,
                                FW_UPDATE_DEBUG_REASON_FLASH_WRITE_FAILED);
            CAN_ReportFlashWriteError();
            return;
        }

        // Формируем подтверждение: номер записанного блока
        state->bytesWritten += bytesToWrite;
        ackData[0] = (uint8_t)(state->currentBlockNum >> 8);
        ackData[1] = (uint8_t)(state->currentBlockNum & 0xFFu);

        // Отправляем extended ACK c номером записанного блока
        FW_SendAck(dstNode, ackData, 2u);
        FW_RecordDebugEvent(state,
                            dstNode,
                            msgId,
                            dataLength,
                            globalPacketIndex,
                            packetNumInBlock,
                            FW_UPDATE_DEBUG_EVENT_BLOCK_WRITTEN,
                            FW_UPDATE_DEBUG_STAGE_RX,
                            FW_UPDATE_DEBUG_REASON_NONE);

        // Переходим к следующему блоку
        state->currentBlockNum++;
        state->packetsInCurrentBlock = 0u;
        state->writeAddress += bytesToWrite;

        // Очищаем буфер для следующего блока
        memset(state->blockBuffer, 0xFF, sizeof(state->blockBuffer));

        /*
         * Если записали все блоки - завершаем обновление.
         *
         * Сначала общий CRC-gate для master и slave. Развилка
         * разная только после успешной проверки:
         * - master: записываем metadata и ставим update-flag;
         * - slave: записываем metadata о том, что в общем слоте лежит slave image.
         */
        if (state->currentBlockNum >= state->totalBlocksExpected)
        {
            state->updateInProgress = false;
            activeFwState = NULL;
            FW_RecordDebugEvent(state,
                                dstNode,
                                msgId,
                                dataLength,
                                globalPacketIndex,
                                packetNumInBlock,
                                FW_UPDATE_DEBUG_EVENT_SESSION_DONE,
                                FW_UPDATE_DEBUG_STAGE_DONE,
                                FW_UPDATE_DEBUG_REASON_NONE);

            crc32 = 0u;
            iar_crc32 = 0u;

            if (!FW_VerifyStoredImageCrc(state, &calculatedCrc, &storedCrc))
            {
                crc32 = calculatedCrc;
                iar_crc32 = storedCrc;

                FW_RecordDebugEvent(state,
                                    dstNode,
                                    msgId,
                                    dataLength,
                                    globalPacketIndex,
                                    packetNumInBlock,
                                    FW_UPDATE_DEBUG_EVENT_FLASH_ERROR,
                                    FW_UPDATE_DEBUG_STAGE_ERROR,
                                    FW_UPDATE_DEBUG_REASON_CRC_FAILED);
                CAN_ReportFlashWriteError();
            }
            else if (state == &masterFwState)
            {
                crc32 = calculatedCrc;
                iar_crc32 = storedCrc;
                if (Set_Update_Flag(state->imageSizeBytes) != HAL_OK)
                {
                    FW_RecordDebugEvent(state,
                                        dstNode,
                                        msgId,
                                        dataLength,
                                        globalPacketIndex,
                                        packetNumInBlock,
                                        FW_UPDATE_DEBUG_EVENT_METADATA_FAIL,
                                        FW_UPDATE_DEBUG_STAGE_ERROR,
                                        FW_UPDATE_DEBUG_REASON_METADATA_FAILED);
                    CAN_ReportFlashWriteError();
                }
            }
            else if (state == &slaveFwState)
            {
                crc32 = calculatedCrc;
                iar_crc32 = storedCrc;
                if (FW_WriteStoredImageInfo(FW_IMAGE_KIND_SLAVE,
                                            state->imageSizeBytes,
                                            FLASH_UPDATE_FLAG_CLEAR_VALUE) != HAL_OK)
                {
                    FW_RecordDebugEvent(state,
                                        dstNode,
                                        msgId,
                                        dataLength,
                                        globalPacketIndex,
                                        packetNumInBlock,
                                        FW_UPDATE_DEBUG_EVENT_METADATA_FAIL,
                                        FW_UPDATE_DEBUG_STAGE_ERROR,
                                        FW_UPDATE_DEBUG_REASON_METADATA_FAILED);
                    CAN_ReportFlashWriteError();
                }
#if (SLAVE_AUTO_UPDATE_ENABLE != 0u)
                else
                {
                    FW_RecordDebugEvent(state,
                                        dstNode,
                                        msgId,
                                        dataLength,
                                        globalPacketIndex,
                                        packetNumInBlock,
                                        FW_UPDATE_DEBUG_EVENT_METADATA_OK,
                                        FW_UPDATE_DEBUG_STAGE_STORED,
                                        FW_UPDATE_DEBUG_REASON_NONE);

                    if (Slave_RequestAutoConnectedUpdate())
                    {
                        FW_RecordDebugEvent(state,
                                            dstNode,
                                            msgId,
                                            dataLength,
                                            globalPacketIndex,
                                            packetNumInBlock,
                                            FW_UPDATE_DEBUG_EVENT_AUTOSTART_OK,
                                            FW_UPDATE_DEBUG_STAGE_SLAVE_START,
                                            FW_UPDATE_DEBUG_REASON_NONE);
                    }
                    else
                    {
                        FW_RecordDebugEvent(state,
                                            dstNode,
                                            msgId,
                                            dataLength,
                                            globalPacketIndex,
                                            packetNumInBlock,
                                            FW_UPDATE_DEBUG_EVENT_AUTOSTART_FAIL,
                                            FW_UPDATE_DEBUG_STAGE_ERROR,
                                            FW_UPDATE_DEBUG_REASON_AUTO_REQUEST_FAILED);
                    }
                }
#else
                else
                {
                    FW_RecordDebugEvent(state,
                                        dstNode,
                                        msgId,
                                        dataLength,
                                        globalPacketIndex,
                                        packetNumInBlock,
                                        FW_UPDATE_DEBUG_EVENT_METADATA_OK,
                                        FW_UPDATE_DEBUG_STAGE_STORED,
                                        FW_UPDATE_DEBUG_REASON_NONE);
                }
#endif
            }
        }
    }
}

/*
 * Фиксация готовности новой master firmware к применению.
 *
 * Что делает:
 * - пишет во flash metadata типа MAST и размер образа;
 * - ставит update-flag 0x1111;
 * - после успешной записи инициирует reset.
 *
 */
HAL_StatusTypeDef Set_Update_Flag(uint32_t imageSizeBytes)
{
    HAL_StatusTypeDef status = FW_WriteStoredImageInfo(FW_IMAGE_KIND_MASTER,
                                                       imageSizeBytes,
                                                       FLASH_UPDATE_FLAG_SET_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_NVIC_SystemReset();
    return status;
}
