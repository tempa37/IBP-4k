#ifndef DIAGNOSTIC_LOG_H
#define DIAGNOSTIC_LOG_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Журнал диагностических ошибок во внутренней Flash STM32F427.
 *
 * Под журнал выделены только сектора 10 и 11:
 *   sector 10: 0x080C0000..0x080DFFFF, 128 KiB
 *   sector 11: 0x080E0000..0x080FFFFF, 128 KiB
 *
 * Сектор 12 не используется намеренно: там лежит metadata принятой прошивки,
 * и firmware_update.c стирает этот сектор целиком при обновлении metadata.
 */
#define DIAG_LOG_START_ADDR              0x080C0000UL
#define DIAG_LOG_END_ADDR                0x080FFFFFUL
#define DIAG_LOG_AREA_SIZE_BYTES         (DIAG_LOG_END_ADDR - DIAG_LOG_START_ADDR + 1UL)
#define DIAG_LOG_SECTOR_SIZE_BYTES       (128UL * 1024UL)
#define DIAG_LOG_SECTOR_RECORD_COUNT     (DIAG_LOG_SECTOR_SIZE_BYTES / DIAG_LOG_RECORD_SIZE_BYTES)
#define DIAG_LOG_RECORD_COUNT            (DIAG_LOG_AREA_SIZE_BYTES / DIAG_LOG_RECORD_SIZE_BYTES)

/*
 * 'DLOG' в little-endian памяти выглядит как байты:
 *   44 4C 4F 47
 */
#define DIAG_LOG_MAGIC                   0x474F4C44UL

/* Коды ошибок журнала: 0x00..0x04 из ТЗ, 0x05 добавлен для фиксации старта после reset. */
#define DIAG_LOG_ERROR_NONE              0x00u
#define DIAG_LOG_ERROR_CAN_BUS_OFF       0x01u
#define DIAG_LOG_ERROR_WDG_RESET         0x02u
#define DIAG_LOG_ERROR_FLASH_CRC         0x03u
#define DIAG_LOG_ERROR_FLASH_WRITE       0x04u
#define DIAG_LOG_ERROR_STARTUP_RESET     0x05u

/* Тип события: сейчас пишем именно факт возникновения диагностической ошибки. */
#define DIAG_LOG_EVENT_ERROR             0x01u

/* Канал 0..3 используется для ошибок конкретного ПСИП, 0xFF - глобальная ошибка. */
#define DIAG_LOG_CHANNEL_GLOBAL          0xFFu

/* Источник показывает, откуда пришло событие, а error_code - какой код из ТЗ записан. */
#define DIAG_LOG_SOURCE_CAN              0x01u
#define DIAG_LOG_SOURCE_WDG              0x02u
#define DIAG_LOG_SOURCE_FLASH            0x03u
#define DIAG_LOG_SOURCE_RESET            0x04u

/* flags для DIAG_LOG_SOURCE_RESET: нормализованные RCC reset flags на момент старта. */
#define DIAG_LOG_RESET_FLAG_BOR          (1UL << 0)
#define DIAG_LOG_RESET_FLAG_PIN          (1UL << 1)
#define DIAG_LOG_RESET_FLAG_POR          (1UL << 2)
#define DIAG_LOG_RESET_FLAG_SOFTWARE     (1UL << 3)
#define DIAG_LOG_RESET_FLAG_IWDG         (1UL << 4)
#define DIAG_LOG_RESET_FLAG_WWDG         (1UL << 5)
#define DIAG_LOG_RESET_FLAG_LOW_POWER    (1UL << 6)

/*
 * Одна запись занимает ровно 32 байта. Поля сгруппированы по размеру, чтобы
 * компилятор не вставил скрытую дыру перед crc32 и не раздул запись.
 */
typedef struct
{
    uint32_t magic;                 /* Сигнатура DIAG_LOG_MAGIC: слот занят нашей записью. */
    uint32_t sequence;              /* Монотонный номер записи; по нему ищем последнюю запись. */
    uint32_t flags;                 /* Дополнительные флаги события, например CAN_ERROR_LOG_FLAG_BUS_OFF. */
    uint32_t detail;                /* Сырое значение источника: HAL_CAN_ErrorCode, reset-флаги, код HAL и т.п. */
    uint16_t size;                  /* sizeof(DiagnosticFlashRecord_t), защита от чтения чужого формата. */
    uint16_t timestamp_hours;       /* Время наработки в часах на момент фиксации ошибки. */
    uint16_t can_busoff_counter;    /* Счетчик входов CAN-контроллера в Bus Off. */
    uint16_t wdg_reset_counter;     /* Счетчик сбросов по watchdog из ТЗ. */
    uint8_t error_code;             /* Код ошибки из таблицы 4.10. */
    uint8_t event_type;             /* Тип события, сейчас DIAG_LOG_EVENT_ERROR. */
    uint8_t channel;                /* Номер канала ПСИП 0..3 или DIAG_LOG_CHANNEL_GLOBAL. */
    uint8_t source;                 /* Источник события: CAN, WDG, Flash. */
    uint32_t crc32;                 /* CRC32 по всем полям записи до crc32. */
} DiagnosticFlashRecord_t;

#define DIAG_LOG_RECORD_SIZE_BYTES       ((uint32_t)sizeof(DiagnosticFlashRecord_t))
#define DIAG_LOG_RECORD_CHUNK_SIZE_BYTES 8u
#define DIAG_LOG_RECORD_CHUNK_COUNT      (DIAG_LOG_RECORD_SIZE_BYTES / DIAG_LOG_RECORD_CHUNK_SIZE_BYTES)
#define DIAG_LOG_INVALID_RECORD_INDEX    0xFFFFu

typedef struct
{
    uint16_t can_busoff_counter;
    uint16_t wdg_reset_counter;
    uint16_t last_error_timestamp_hours;
    uint8_t last_error_code;
} DiagnosticLogCounters_t;

typedef struct
{
    uint16_t record_capacity;       /* Сколько физических слотов журнала доступно во Flash. */
    uint16_t valid_record_count;    /* Сколько слотов сейчас содержат валидные записи с правильным CRC. */
    uint16_t last_record_index;     /* Физический индекс последней записи или 0xFFFF, если журнал пуст. */
    uint8_t record_size_bytes;      /* Размер одной записи; сейчас 32 байта. */
    uint8_t chunks_per_record;      /* Сколько 8-байтных CAN-ответов нужно для выгрузки одной записи. */
} DiagnosticLogExportInfo_t;

/* Инициализирует RAM-состояние диагностического журнала по содержимому flash. */
void DiagnosticLog_Init(void);

/* Возвращает признак найденной CRC-ошибки в журнале. */
bool DiagnosticLog_HadCrcError(void);

/* Проверяет CRC последней сохраненной записи журнала. */
bool DiagnosticLog_CheckLastRecordCrc(void);

/* Возвращает сохраненные диагностические счетчики. */
void DiagnosticLog_GetCounters(DiagnosticLogCounters_t *counters);

/* Возвращает RAM-копию последней валидной записи журнала. */
const DiagnosticFlashRecord_t *DiagnosticLog_GetLastRecord(void);

/* Формирует сведения для выгрузки журнала по CAN. */
void DiagnosticLog_GetExportInfo(DiagnosticLogExportInfo_t *info);

/* Читает 8-байтный фрагмент записи журнала для передачи по CAN. */
bool DiagnosticLog_ReadRecordChunk(uint16_t record_index,
                                   uint8_t chunk_index,
                                   uint8_t chunk_data[DIAG_LOG_RECORD_CHUNK_SIZE_BYTES]);

/* Добавляет новое событие в кольцевой flash-журнал диагностики. */
HAL_StatusTypeDef DiagnosticLog_RecordEvent(uint8_t error_code,
                                            uint8_t event_type,
                                            uint8_t channel,
                                            uint8_t source,
                                            uint32_t flags,
                                            uint32_t detail,
                                            uint16_t timestamp_hours,
                                            uint16_t can_busoff_counter,
                                            uint16_t wdg_reset_counter);

#endif /* DIAGNOSTIC_LOG_H */
