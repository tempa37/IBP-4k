#ifndef UART_H
#define UART_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "modbuc.h"


/* Размер буфера приёма UART */
#define UART_RX_CHUNK_SIZE              (MODBUS_MAX_FRAME_SIZE)

/* Перечень используемых UART-каналов */
typedef enum
{
    UART_CHANNEL_8 = 0u,
    UART_CHANNEL_7,
    UART_CHANNEL_4,
    UART_CHANNEL_5,
    UART_CHANNEL_COUNT  
} UartChannel_t;

/* Структура для хранения контекста UART */
typedef struct
{
    UART_HandleTypeDef *handle;
    osMutexId mutex;
    uint8_t rxTransferBuffer[UART_RX_CHUNK_SIZE];
    uint8_t rxBuffer[UART_RX_CHUNK_SIZE];
    size_t rxLength;
    volatile bool dataReady;
    volatile bool errorDetected;
    volatile uint32_t errorCode;
    volatile bool reinit_needed;
} UartContext_t;

/* Конфигурация UART */
typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
} UartHwCfg;

/* Глобальные переменные UART (объявление) */
extern UartContext_t uartContexts[UART_CHANNEL_COUNT];
extern const uint8_t uartIndexToNumber[UART_CHANNEL_COUNT];
extern const UartHwCfg myuart;

/* Флаги ошибок UART */
extern uint8_t uart8_err;
extern uint8_t uart7_err;
extern uint8_t uart4_err;
extern uint8_t uart5_err;
extern volatile uint32_t uart_debug_error_state;
extern volatile uint32_t uart_debug_error_sr;
extern volatile uint32_t uart_debug_error_dr;
extern volatile uint32_t uart_debug_error_count;
extern volatile uint8_t uart_debug_error_uart;

/* Функции UART */
void UART_Init(void);
void UART_StartAllReceptions(void);
UartContext_t *Uart_GetContext(UART_HandleTypeDef *handle);
void Uart_StartReception(UartContext_t *context);
void Uart_ErrorRecovery(void);

/* Инициализация UART */
void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);

#endif /* UART_H */
