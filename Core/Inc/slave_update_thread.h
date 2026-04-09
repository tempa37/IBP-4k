#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "modbuc.h"
#include "uart.h"
/*
 * Машина состояний прошивки слейва по STM32 bootloader over UART.
 *
 * Важная роль этого модуля:
 * - он НЕ принимает образ по CAN;
 * - он только берёт уже сохранённый образ из общего staging-слота мастера
 *   и перекладывает его в конкретный slave по UART.
 *
 * То есть хранение образа и отправка образа - это две разные стадии.
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

/*
 * Контекст одной UART-сессии прошивки слейва.
 *
 * Здесь хранится именно процесс отправки во внешний slave:
 * текущая фаза, адрес, сколько уже отправили, сколько ещё осталось,
 * буфер текущего блока и служебные поля по таймаутам.
 */
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
#define SLAVE_FW_SEND_ENABLE        0u     // 0 - полностью отключить отправку слейв-прошивки по UART
typedef struct {
    volatile uint8_t ack;
    volatile uint8_t need_update;         // Запрос на запуск отправки уже сохранённого slave image
} SlaveDevice_t;

typedef struct {
    SlaveDevice_t device[UART_CHANNEL_COUNT];
    volatile uint8_t os_in_flash;         // В общем staging-слоте лежит образ слейва
    volatile uint32_t os_size_bytes;      // Размер сохранённого slave image в байтах
} SlaveSystem_t;

/* Глобальная переменная, определена в main.c */
extern SlaveSystem_t slave;

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



void Slave_SendBootloaderCommand(uint8_t slave_num, const uint8_t *data, uint8_t length);
bool Slave_CheckACK(uint8_t slave_num);
void Slave_NextDataBlock(SlaveUpdateContext_t *ctx);
void Slave_UpdateProcess(void);
