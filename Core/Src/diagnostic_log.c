#include "diagnostic_log.h"

#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#include <stddef.h>
#include <string.h>

/* Проверка размера записи на этапе компиляции. Если размер стал не 32 байта,
 * значит кто-то изменил структуру и сломал расчет емкости кольцевого журнала. */
typedef char DiagnosticFlashRecord_SizeMustBe32[(sizeof(DiagnosticFlashRecord_t) == 32u) ? 1 : -1];

static bool diagnosticLogInitialized = false;
static bool diagnosticLogCrcErrorDetected = false;
static uint32_t diagnosticLogNextIndex = 0u;
static uint32_t diagnosticLogNextSequence = 1u;
static uint32_t diagnosticLogLastRecordIndex = 0u;
static bool diagnosticLogHasLastRecord = false;
static DiagnosticFlashRecord_t diagnosticLogLastRecord = {0};
static DiagnosticLogCounters_t diagnosticLogCounters = {0};

static uint32_t DiagnosticLog_CalcCrc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    while (length > 0u)
    {
        crc ^= *data;
        ++data;
        --length;

        for (uint8_t bit = 0u; bit < 8u; ++bit)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t DiagnosticLog_RecordAddress(uint32_t index)
{
    return DIAG_LOG_START_ADDR + (index * DIAG_LOG_RECORD_SIZE_BYTES);
}

static bool DiagnosticLog_IsRangeErased(uint32_t address, uint32_t length)
{
    const uint32_t *word = (const uint32_t *)address;
    uint32_t wordCount = length / sizeof(uint32_t);

    for (uint32_t index = 0u; index < wordCount; ++index)
    {
        if (word[index] != 0xFFFFFFFFUL)
        {
            return false;
        }
    }

    return true;
}

static bool DiagnosticLog_IsSlotErased(uint32_t index)
{
    return DiagnosticLog_IsRangeErased(DiagnosticLog_RecordAddress(index),
                                       DIAG_LOG_RECORD_SIZE_BYTES);
}

static bool DiagnosticLog_IsSectorErased(uint32_t sectorIndex)
{
    uint32_t sectorAddress = DIAG_LOG_START_ADDR + (sectorIndex * DIAG_LOG_SECTOR_SIZE_BYTES);

    return DiagnosticLog_IsRangeErased(sectorAddress, DIAG_LOG_SECTOR_SIZE_BYTES);
}

static uint32_t DiagnosticLog_SectorIndexForRecord(uint32_t recordIndex)
{
    return recordIndex / DIAG_LOG_SECTOR_RECORD_COUNT;
}

static uint32_t DiagnosticLog_FirstRecordIndexOfSector(uint32_t sectorIndex)
{
    return sectorIndex * DIAG_LOG_SECTOR_RECORD_COUNT;
}

static uint32_t DiagnosticLog_NextSectorFirstRecord(uint32_t sectorIndex)
{
    return DiagnosticLog_FirstRecordIndexOfSector((sectorIndex + 1u) % 2u);
}

static uint32_t DiagnosticLog_NextSequenceValue(uint32_t sequence)
{
    return (sequence == 0xFFFFFFFFUL) ? 1u : (sequence + 1u);
}

static bool DiagnosticLog_IsRecordHeaderValid(const DiagnosticFlashRecord_t *record)
{
    if (record == NULL)
    {
        return false;
    }

    return ((record->magic == DIAG_LOG_MAGIC) &&
            (record->size == (uint16_t)sizeof(DiagnosticFlashRecord_t)));
}

static bool DiagnosticLog_IsRecordValid(const DiagnosticFlashRecord_t *record)
{
    uint32_t calculatedCrc;

    if (!DiagnosticLog_IsRecordHeaderValid(record))
    {
        return false;
    }

    calculatedCrc = DiagnosticLog_CalcCrc32((const uint8_t *)record,
                                            (uint32_t)offsetof(DiagnosticFlashRecord_t, crc32));

    return (calculatedCrc == record->crc32);
}

static void DiagnosticLog_LoadRecordToRuntimeState(uint32_t recordIndex,
                                                   const DiagnosticFlashRecord_t *record)
{
    diagnosticLogLastRecord = *record;
    diagnosticLogLastRecordIndex = recordIndex;
    diagnosticLogHasLastRecord = true;
    diagnosticLogCounters.can_busoff_counter = record->can_busoff_counter;
    diagnosticLogCounters.wdg_reset_counter = record->wdg_reset_counter;
    diagnosticLogCounters.last_error_timestamp_hours = record->timestamp_hours;
    diagnosticLogCounters.last_error_code = record->error_code;
}

static bool DiagnosticLog_FindLastWrittenIndexInSector(uint32_t sectorIndex,
                                                       uint32_t *recordIndex)
{
    uint32_t firstRecord = DiagnosticLog_FirstRecordIndexOfSector(sectorIndex);
    uint32_t low = 0u;
    uint32_t high = DIAG_LOG_SECTOR_RECORD_COUNT;

    if (recordIndex == NULL)
    {
        return false;
    }

    while (low < high)
    {
        uint32_t middle = low + ((high - low) / 2u);
        uint32_t middleRecord = firstRecord + middle;

        if (DiagnosticLog_IsSlotErased(middleRecord))
        {
            high = middle;
        }
        else
        {
            low = middle + 1u;
        }
    }

    if (low == 0u)
    {
        return false;
    }

    *recordIndex = firstRecord + low - 1u;
    return true;
}

static uint32_t DiagnosticLog_SelectNextIndex(uint32_t latestIndex)
{
    uint32_t candidate = (latestIndex + 1u) % DIAG_LOG_RECORD_COUNT;
    uint32_t latestSector = DiagnosticLog_SectorIndexForRecord(latestIndex);
    uint32_t candidateSector = DiagnosticLog_SectorIndexForRecord(candidate);

    /*
     * Если следующий слот в том же секторе уже не пустой, значит сектор
     * содержит мусор/старый хвост после сбоя. Не стираем сектор с последней
     * валидной записью, а уходим в соседний сектор.
     */
    if (!DiagnosticLog_IsSlotErased(candidate) && (candidateSector == latestSector))
    {
        candidate = DiagnosticLog_NextSectorFirstRecord(latestSector);
    }

    return candidate;
}

static HAL_StatusTypeDef DiagnosticLog_EraseSector(uint32_t sectorIndex)
{
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0u;
    HAL_StatusTypeDef status;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = (sectorIndex == 0u) ? FLASH_SECTOR_10 : FLASH_SECTOR_11;
    eraseInit.NbSectors = 1u;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    return status;
}

static HAL_StatusTypeDef DiagnosticLog_PrepareSlot(uint32_t recordIndex)
{
    uint32_t sectorIndex = DiagnosticLog_SectorIndexForRecord(recordIndex);
    uint32_t firstRecordInSector = DiagnosticLog_FirstRecordIndexOfSector(sectorIndex);

    /*
     * При входе в сектор стираем его целиком, если там есть старые записи.
     * Это и есть секторное кольцо: сектор 10 -> сектор 11 -> сектор 10.
     */
    if ((recordIndex == firstRecordInSector) && !DiagnosticLog_IsSectorErased(sectorIndex))
    {
        return DiagnosticLog_EraseSector(sectorIndex);
    }

    /*
     * Если конкретный слот не пустой, поверх него писать нельзя: Flash умеет
     * переводить биты только из 1 в 0. Стираем весь сектор, иначе получим
     * нечитаемую кашу вместо журнала.
     */
    if (!DiagnosticLog_IsSlotErased(recordIndex))
    {
        return DiagnosticLog_EraseSector(sectorIndex);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef DiagnosticLog_WriteRecord(uint32_t address,
                                                   const DiagnosticFlashRecord_t *record)
{
    const uint32_t *word = (const uint32_t *)record;
    HAL_StatusTypeDef status = HAL_OK;

    HAL_FLASH_Unlock();

    for (uint32_t index = 0u; index < (DIAG_LOG_RECORD_SIZE_BYTES / sizeof(uint32_t)); ++index)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   address + (index * sizeof(uint32_t)),
                                   word[index]);
        if (status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    return status;
}

void DiagnosticLog_Init(void)
{
    bool foundRecord = false;
    bool foundWrittenRecord = false;
    uint32_t latestIndex = 0u;
    uint32_t latestSequence = 0u;
    uint32_t latestWrittenIndex = 0u;
    uint32_t fallbackWrittenIndex = 0u;
    const DiagnosticFlashRecord_t *latestRecord = NULL;

    memset(&diagnosticLogLastRecord, 0, sizeof(diagnosticLogLastRecord));
    memset(&diagnosticLogCounters, 0, sizeof(diagnosticLogCounters));
    diagnosticLogCrcErrorDetected = false;
    diagnosticLogNextIndex = 0u;
    diagnosticLogNextSequence = 1u;
    diagnosticLogLastRecordIndex = 0u;
    diagnosticLogHasLastRecord = false;

    for (uint32_t sectorIndex = 0u; sectorIndex < 2u; ++sectorIndex)
    {
        uint32_t candidateIndex = 0u;
        uint32_t validIndex = 0u;
        const DiagnosticFlashRecord_t *candidateRecord;
        const DiagnosticFlashRecord_t *validRecord = NULL;

        if (!DiagnosticLog_FindLastWrittenIndexInSector(sectorIndex, &candidateIndex))
        {
            continue;
        }

        if (!foundWrittenRecord)
        {
            foundWrittenRecord = true;
            fallbackWrittenIndex = candidateIndex;
        }

        candidateRecord =
            (const DiagnosticFlashRecord_t *)DiagnosticLog_RecordAddress(candidateIndex);

        if (DiagnosticLog_IsRecordValid(candidateRecord))
        {
            validIndex = candidateIndex;
            validRecord = candidateRecord;
        }
        else
        {
            uint32_t firstRecordInSector = DiagnosticLog_FirstRecordIndexOfSector(sectorIndex);

            diagnosticLogCrcErrorDetected = true;

            if (candidateIndex > firstRecordInSector)
            {
                const DiagnosticFlashRecord_t *previousRecord =
                    (const DiagnosticFlashRecord_t *)DiagnosticLog_RecordAddress(candidateIndex - 1u);

                if (DiagnosticLog_IsRecordValid(previousRecord))
                {
                    validIndex = candidateIndex - 1u;
                    validRecord = previousRecord;
                }
            }
        }

        if (validRecord != NULL)
        {
            if (!foundRecord || (validRecord->sequence > latestSequence))
            {
                foundRecord = true;
                latestSequence = validRecord->sequence;
                latestIndex = validIndex;
                latestWrittenIndex = candidateIndex;
                latestRecord = validRecord;
            }
        }
    }

    if (foundRecord && (latestRecord != NULL))
    {
        DiagnosticLog_LoadRecordToRuntimeState(latestIndex, latestRecord);
        diagnosticLogNextIndex = DiagnosticLog_SelectNextIndex(latestWrittenIndex);
        diagnosticLogNextSequence = DiagnosticLog_NextSequenceValue(latestSequence);
    }
    else if (foundWrittenRecord)
    {
        diagnosticLogNextIndex = DiagnosticLog_SelectNextIndex(fallbackWrittenIndex);
    }

    diagnosticLogInitialized = true;
}

bool DiagnosticLog_HadCrcError(void)
{
    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    return diagnosticLogCrcErrorDetected;
}

bool DiagnosticLog_CheckLastRecordCrc(void)
{
    const DiagnosticFlashRecord_t *record;
    bool valid = true;

    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    if (diagnosticLogHasLastRecord)
    {
        record = (const DiagnosticFlashRecord_t *)DiagnosticLog_RecordAddress(diagnosticLogLastRecordIndex);
        valid = DiagnosticLog_IsRecordValid(record);
        diagnosticLogCrcErrorDetected = !valid;
    }

    return valid;
}

void DiagnosticLog_GetCounters(DiagnosticLogCounters_t *counters)
{
    if (counters == NULL)
    {
        return;
    }

    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    *counters = diagnosticLogCounters;
}

const DiagnosticFlashRecord_t *DiagnosticLog_GetLastRecord(void)
{
    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    return &diagnosticLogLastRecord;
}

void DiagnosticLog_GetExportInfo(DiagnosticLogExportInfo_t *info)
{
    bool foundLatestRecord = false;
    uint32_t latestSequence = 0u;

    if (info == NULL)
    {
        return;
    }

    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    memset(info, 0, sizeof(*info));
    info->record_capacity = (uint16_t)DIAG_LOG_RECORD_COUNT;
    info->record_size_bytes = (uint8_t)DIAG_LOG_RECORD_SIZE_BYTES;
    info->chunks_per_record = (uint8_t)DIAG_LOG_RECORD_CHUNK_COUNT;
    info->last_record_index = DIAG_LOG_INVALID_RECORD_INDEX;

    /*
     * Для выгрузки журнала по CAN отдаем не указатель на внутреннее состояние,
     * а короткую сводку: ПК сам решает, какие физические слоты читать дальше.
     */
    for (uint32_t index = 0u; index < DIAG_LOG_RECORD_COUNT; ++index)
    {
        const DiagnosticFlashRecord_t *record =
            (const DiagnosticFlashRecord_t *)DiagnosticLog_RecordAddress(index);

        if (!DiagnosticLog_IsRecordValid(record))
        {
            continue;
        }

        if (info->valid_record_count < 0xFFFFu)
        {
            ++info->valid_record_count;
        }

        if (!foundLatestRecord || (record->sequence > latestSequence))
        {
            foundLatestRecord = true;
            latestSequence = record->sequence;
            info->last_record_index = (uint16_t)index;
        }
    }
}

bool DiagnosticLog_ReadRecordChunk(uint16_t record_index,
                                   uint8_t chunk_index,
                                   uint8_t chunk_data[DIAG_LOG_RECORD_CHUNK_SIZE_BYTES])
{
    const DiagnosticFlashRecord_t *record;
    const uint8_t *recordBytes;
    uint32_t byteOffset;

    if (chunk_data == NULL)
    {
        return false;
    }

    memset(chunk_data, 0xFF, DIAG_LOG_RECORD_CHUNK_SIZE_BYTES);

    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    if ((record_index >= DIAG_LOG_RECORD_COUNT) ||
        (chunk_index >= DIAG_LOG_RECORD_CHUNK_COUNT))
    {
        return false;
    }

    record = (const DiagnosticFlashRecord_t *)DiagnosticLog_RecordAddress(record_index);
    if (!DiagnosticLog_IsRecordValid(record))
    {
        return false;
    }

    /*
     * Одна запись 32 байта, CAN несет 8 байт. Поэтому запись читается
     * четырьмя чанками: 0 -> байты 0..7, 1 -> 8..15, 2 -> 16..23, 3 -> 24..31.
     */
    recordBytes = (const uint8_t *)record;
    byteOffset = (uint32_t)chunk_index * DIAG_LOG_RECORD_CHUNK_SIZE_BYTES;
    memcpy(chunk_data, &recordBytes[byteOffset], DIAG_LOG_RECORD_CHUNK_SIZE_BYTES);

    return true;
}

HAL_StatusTypeDef DiagnosticLog_RecordEvent(uint8_t error_code,
                                            uint8_t event_type,
                                            uint8_t channel,
                                            uint8_t source,
                                            uint32_t flags,
                                            uint32_t detail,
                                            uint16_t timestamp_hours,
                                            uint16_t can_busoff_counter,
                                            uint16_t wdg_reset_counter)
{
    DiagnosticFlashRecord_t record = {0};
    uint32_t recordIndex;
    uint32_t recordAddress;
    HAL_StatusTypeDef status;

    if (!diagnosticLogInitialized)
    {
        DiagnosticLog_Init();
    }

    recordIndex = diagnosticLogNextIndex;
    recordAddress = DiagnosticLog_RecordAddress(recordIndex);

    status = DiagnosticLog_PrepareSlot(recordIndex);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Заполняем полную запись. crc32 считается последним и покрывает все
     * остальные поля: при оборванном питании запись просто не пройдет проверку.
     */
    record.magic = DIAG_LOG_MAGIC;
    record.sequence = diagnosticLogNextSequence;
    record.flags = flags;
    record.detail = detail;
    record.size = (uint16_t)sizeof(DiagnosticFlashRecord_t);
    record.timestamp_hours = timestamp_hours;
    record.can_busoff_counter = can_busoff_counter;
    record.wdg_reset_counter = wdg_reset_counter;
    record.error_code = error_code;
    record.event_type = event_type;
    record.channel = channel;
    record.source = source;
    record.crc32 = DiagnosticLog_CalcCrc32((const uint8_t *)&record,
                                           (uint32_t)offsetof(DiagnosticFlashRecord_t, crc32));

    status = DiagnosticLog_WriteRecord(recordAddress, &record);
    if (status != HAL_OK)
    {
        return status;
    }

    DiagnosticLog_LoadRecordToRuntimeState(recordIndex, &record);
    diagnosticLogNextSequence = DiagnosticLog_NextSequenceValue(diagnosticLogNextSequence);
    diagnosticLogNextIndex = (recordIndex + 1u) % DIAG_LOG_RECORD_COUNT;

    return HAL_OK;
}
