#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "modbuc.h"
#include "uart.h"
/**
 * @brief Процесс обновления прошивки слейва по протоколу STM32 Bootloader
 * Реализует алгоритм:
 * 1) Вход в bootloader (запись 0x1234 в регистр 0x0000)
 * 2) Синхронизация (0x7F)
 * 3) Стирание (0x44 + 0xBB)
 * 4) Mass Erase (0xFF 0xFF + 0x00)
 * 5) Команда записи (0x31 + 0xCE)
 * 6-8) Циклически: адрес (0x08 0x00 0x00 0x00 + 0x08) и данные (0xFF + data)
 * 9) На каждый пакет слейв отправляет ACK (0x79)
 */

/* Состояния процесса обновления слейва */
typedef enum {
    SLAVE_UPD_IDLE,                    // Ничего не происходит
    SLAVE_UPD_WAIT_REQUEST,            // Ждём запроса на обновление
    SLAVE_UPD_SEND_ENTER_BOOTLOADER,   // Отправляем команду входа в bootloader
    SLAVE_UPD_WAIT_ACK_BOOTLOADER,     // Ждём ACK после входа в bootloader
    SLAVE_UPD_SEND_SYNC,               // Отправляем синхронизацию 0x7F
    SLAVE_UPD_WAIT_ACK_SYNC,           // Ждём ACK после синхронизации
    SLAVE_UPD_SEND_ERASE,              // Отправляем команду стирания 0x44
    SLAVE_UPD_WAIT_ACK_ERASE,          // Ждём ACK после команды стирания
    SLAVE_UPD_SEND_ERASE_CONFIRM,      // Отправляем подтверждение стирания 0xBB
    SLAVE_UPD_WAIT_ACK_ERASE_CONF,     // Ждём ACK после подтверждения
    SLAVE_UPD_SEND_MASS_ERASE,         // Отправляем Mass Erase 0xFF 0xFF
    SLAVE_UPD_WAIT_ACK_MASS_ERASE,     // Ждём ACK после Mass Erase
    SLAVE_UPD_SEND_MASS_ERASE_CONF,    // Отправляем подтверждение Mass Erase 0x00
    SLAVE_UPD_WAIT_ACK_MASS_ERASE_CONF,// Ждём ACK после подтверждения Mass Erase
    SLAVE_UPD_SEND_WRITE_CMD,          // Отправляем команду записи 0x31
    SLAVE_UPD_WAIT_ACK_WRITE_CMD,      // Ждём ACK после команды записи
    SLAVE_UPD_SEND_WRITE_CONFIRM,      // Отправляем подтверждение команды записи 0xCE
    SLAVE_UPD_WAIT_ACK_WRITE_CONFIRM,  // Ждём ACK после подтверждения
    SLAVE_UPD_SEND_ADDRESS,            // Отправляем адрес 0x08 0x00 0x00 0x00
    SLAVE_UPD_WAIT_ACK_ADDRESS,        // Ждём ACK после адреса
    SLAVE_UPD_SEND_DATA_SIZE,          // Отправляем размер данных (N-1)
    SLAVE_UPD_WAIT_ACK_DATA_SIZE,      // Ждём ACK после размера
    SLAVE_UPD_SEND_DATA,               // Отправляем данные
    SLAVE_UPD_WAIT_ACK_DATA,           // Ждём ACK после данных
    SLAVE_UPD_NEXT_BLOCK,              // Переход к следующему блоку данных
    SLAVE_UPD_DONE,                    // Обновление завершено успешно
    SLAVE_UPD_ERROR                    // Ошибка обновления
} SlaveUpdatePhase_t;

/* Структура для отслеживания процесса обновления */
typedef struct {
    SlaveUpdatePhase_t phase;          // Текущая фаза обновления
    uint8_t slave_number;              // Номер слейва (0-3)
    uint32_t current_address;          // Текущий адрес для записи
    uint32_t bytes_written;            // Количество записанных байт
    uint32_t total_bytes;              // Общее количество байт к записи
    uint8_t retry_count;               // Счётчик повторов
    uint32_t last_command_tick;        // Время последней команды
    uint8_t write_buffer[64];          // Буфер текущих данных для записи
    uint16_t write_buffer_size;        // Размер данных в буфере
    uint8_t ack_received;              // Флаг получения ACK
} SlaveUpdateContext_t;

/* Хранилище контекстов обновления для каждого слейва */
static SlaveUpdateContext_t slave_update_ctx[UART_CHANNEL_COUNT] = {0};

#define SLAVE_UPDATE_TIMEOUT_MS     5000u  // Таймаут ответа слейва
#define SLAVE_UPDATE_RETRY_MAX      3u     // Максимум повторов
#define SLAVE_FW_BLOCK_SIZE         256u   // Размер блока данных для записи
#define SLAVE_OS_SIZE               0x8000u   // 32 KB




/* Определяем константы для bootloader протокола */
#define BOOTLOADER_ACK              0x79u
#define BOOTLOADER_NACK             0x1Fu
#define BOOTLOADER_SYNC             0x7Fu
#define BOOTLOADER_ERASE            0x44u
#define BOOTLOADER_ERASE_CONFIRM    0xBBu
#define BOOTLOADER_MASS_ERASE_CMD   0xFFu
#define BOOTLOADER_MASS_ERASE_CONF  0x00u
#define BOOTLOADER_WRITE            0x31u
#define BOOTLOADER_WRITE_CONFIRM    0xCEu


