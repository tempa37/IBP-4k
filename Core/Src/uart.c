#include "uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* Глобальные переменные UART */
UartContext_t uartContexts[UART_CHANNEL_COUNT] = {0};
const uint8_t uartIndexToNumber[UART_CHANNEL_COUNT] = {8u, 7u, 4u, 5u};

/* Флаги ошибок UART */
uint8_t uart8_err = 0;
uint8_t uart7_err = 0;
uint8_t uart4_err = 0;
uint8_t uart5_err = 0;
volatile uint32_t uart_debug_error_state = 0u;
volatile uint32_t uart_debug_error_sr = 0u;
volatile uint32_t uart_debug_error_dr = 0u;
volatile uint32_t uart_debug_error_count = 0u;
volatile uint8_t uart_debug_error_uart = 0xFFu;


/* Конфигурация UART */
uint8_t swap_crc = 0;

const UartHwCfg myuart = {
    .BaudRate   = 115200u,
    .WordLength = UART_WORDLENGTH_9B,
    .StopBits   = UART_STOPBITS_1,
    .Parity     = UART_PARITY_EVEN
};

/* Внешние переменные из main.c */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

/**
 * @brief Инициализация контекстов UART
 */
void UART_Init(void)
{
    uartContexts[UART_CHANNEL_8].handle = &huart8;
    uartContexts[UART_CHANNEL_7].handle = &huart7;
    uartContexts[UART_CHANNEL_4].handle = &huart4;
    uartContexts[UART_CHANNEL_5].handle = &huart5;
}

/**
 * @brief Запуск приёма для всех UART
 */
void UART_StartAllReceptions(void)
{
    for (size_t index = 0; index < UART_CHANNEL_COUNT; ++index)
    {
        UartContext_t *ctx = &uartContexts[index];
        
        // Пропускаем если не инициализирован
        if (ctx->handle->Instance == NULL) {
            continue;
        }
        
        Uart_StartReception(&uartContexts[index]);
    }
}

/**
 * @brief Поиск контекста UART по указателю на хендл
 */
UartContext_t *Uart_GetContext(UART_HandleTypeDef *handle)
{
    for (size_t index = 0; index < UART_CHANNEL_COUNT; ++index)
    {
        if (uartContexts[index].handle == handle)
        {
            return &uartContexts[index];
        }
    }
    return NULL;
}

/**
 * @brief Перезапуск приёма UART с использованием прерываний
 */
void Uart_StartReception(UartContext_t *context)
{
    if (context == NULL)
    {
        return;
    }
    
    /* Запуск приёма UART до наступления тишины на линии */
    if (HAL_UARTEx_ReceiveToIdle_IT(context->handle, context->rxTransferBuffer, UART_RX_CHUNK_SIZE) != HAL_OK)
    {
        context->errorDetected = true;
        context->errorCode = context->handle->ErrorCode;
        HAL_UART_ErrorCallback(context->handle);
    }
}

/**
 * @brief Обработчик событий приёма UART с фиксацией тишины на линии
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UartContext_t *context = Uart_GetContext(huart);
    if (context == NULL)
    {
        return;
    }

    UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
    size_t copyLength = (Size <= UART_RX_CHUNK_SIZE) ? Size : UART_RX_CHUNK_SIZE;
    memcpy(context->rxBuffer, context->rxTransferBuffer, copyLength);
    if (copyLength < UART_RX_CHUNK_SIZE)
    {
        memset(&context->rxBuffer[copyLength], 0, UART_RX_CHUNK_SIZE - copyLength);
    }
    context->rxLength = copyLength;
    context->dataReady = true;
    taskEXIT_CRITICAL_FROM_ISR(irqState);

    /* Немедленно перезапускаем приём для следующего кадра */
    Uart_StartReception(context);
}

/**
 * @brief Совместимость с обработчиком полного буфера при работе без режима «тишина»
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_RxEventCallback(huart, UART_RX_CHUNK_SIZE);
}

/**
 * @brief Обработчик ошибок UART
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UartContext_t *context = Uart_GetContext(huart);
    if (!context) return;
    uart_debug_error_state = huart->ErrorCode;
    uart_debug_error_sr = huart->Instance->SR;
    uart_debug_error_dr = huart->Instance->DR;
    uart_debug_error_count++;
    if (huart == &huart8) uart_debug_error_uart = 8u;
    else if (huart == &huart7) uart_debug_error_uart = 7u;
    else if (huart == &huart4) uart_debug_error_uart = 4u;
    else if (huart == &huart5) uart_debug_error_uart = 5u;
    else uart_debug_error_uart = 0u;

    // Сохраняем код ошибки
    context->errorCode = huart->ErrorCode;
    context->errorDetected = true; 
    context->reinit_needed = true; // Сигнализируем задаче, что нужен рестарт
    context->dataReady = false;

    // Сброс флагов ошибок на уровне регистров (для F4/F7 это чтение SR и DR)
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    
    // ВАЖНО: Принудительно сбрасываем прием в HAL, чтобы сбросить состояние BUSY
    HAL_UART_AbortReceive_IT(huart); 
}

/**
 * @brief Восстановление после ошибок UART
 */
void Uart_CheckAndRecover(UartContext_t *ctx)
{
    if (ctx->reinit_needed)
    {
        taskENTER_CRITICAL();
        ctx->reinit_needed = false;
        taskEXIT_CRITICAL();

        // Жесткий сброс приема перед перезапуском
        HAL_UART_AbortReceive(ctx->handle);
        
        // Небольшая задержка или очистка буфера
        memset(ctx->rxTransferBuffer, 0, UART_RX_CHUNK_SIZE);

        // Перезапуск
        Uart_StartReception(ctx);
    }
}


/**
 * @brief UART4 Initialization Function
 */
void MX_UART4_Init(void)
{
    extern UART_HandleTypeDef huart4;
    
    huart4.Instance = UART4;
    huart4.Init.BaudRate = myuart.BaudRate;
    huart4.Init.WordLength = myuart.WordLength;
    huart4.Init.StopBits = myuart.StopBits;
    huart4.Init.Parity = myuart.Parity;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief UART5 Initialization Function
 */
void MX_UART5_Init(void)
{
    extern UART_HandleTypeDef huart5;
    
    huart5.Instance = UART5;
    huart5.Init.BaudRate = myuart.BaudRate;
    huart5.Init.WordLength = myuart.WordLength;
    huart5.Init.StopBits = myuart.StopBits;
    huart5.Init.Parity = myuart.Parity;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief UART7 Initialization Function
 */
void MX_UART7_Init(void)
{
    extern UART_HandleTypeDef huart7;
    
    huart7.Instance = UART7;
    huart7.Init.BaudRate = myuart.BaudRate;
    huart7.Init.WordLength = myuart.WordLength;
    huart7.Init.StopBits = myuart.StopBits;
    huart7.Init.Parity = myuart.Parity;
    huart7.Init.Mode = UART_MODE_TX_RX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart7) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief UART8 Initialization Function
 */
void MX_UART8_Init(void)
{
    extern UART_HandleTypeDef huart8;
    
    huart8.Instance = UART8;
    huart8.Init.BaudRate = myuart.BaudRate;
    huart8.Init.WordLength = myuart.WordLength;
    huart8.Init.StopBits = myuart.StopBits;
    huart8.Init.Parity = myuart.Parity;
    huart8.Init.Mode = UART_MODE_TX_RX;
    huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart8) != HAL_OK)
    {
        Error_Handler();
    }
}
