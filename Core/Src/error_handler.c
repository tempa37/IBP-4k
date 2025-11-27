/**
  ******************************************************************************
  * @file           : error_handler.c
  * @brief          : Implementation of enhanced error handling
  ******************************************************************************
  */

#include "error_handler.h"
#include "main.h"
#include <string.h>

/* Private function prototypes */
static void LogError(ErrorLogEntry_t *log, uint8_t *index, uint32_t errorCode, uint8_t errorType);
static uint32_t GetExponentialDelay(uint32_t baseDelay, uint32_t attemptCount);

/**
  * @brief  Initialize UART error context
  * @param  ctx: Pointer to UART error context
  * @retval None
  */
void ErrorHandler_InitUart(UartErrorContext_t *ctx)
{
    if (ctx == NULL) return;
    
    memset(ctx, 0, sizeof(UartErrorContext_t));
    ctx->needsReinit = false;
    ctx->nextRecoveryTime = 0;
}

/**
  * @brief  Initialize CAN error context
  * @param  ctx: Pointer to CAN error context
  * @retval None
  */
void ErrorHandler_InitCan(CanErrorContext_t *ctx)
{
    if (ctx == NULL) return;
    
    memset(ctx, 0, sizeof(CanErrorContext_t));
    ctx->needsReinit = false;
    ctx->busOffDetected = false;
    ctx->nextRecoveryTime = 0;
}

/**
  * @brief  Handle UART error
  * @param  ctx: Pointer to UART error context
  * @param  huart: Pointer to UART handle
  * @retval None
  */
void ErrorHandler_UartError(UartErrorContext_t *ctx, UART_HandleTypeDef *huart)
{
    if (ctx == NULL || huart == NULL) return;
    
    uint32_t errorCode = huart->ErrorCode;
    ctx->stats.totalErrors++;
    ctx->stats.consecutiveErrors++;
    ctx->stats.lastErrorTick = HAL_GetTick();
    
    /* Classify and count specific errors */
    if (errorCode & HAL_UART_ERROR_PE) {
        ctx->stats.parityErrors++;
        ErrorHandler_LogUartError(ctx, errorCode, 1);
    }
    if (errorCode & HAL_UART_ERROR_FE) {
        ctx->stats.framingErrors++;
        ErrorHandler_LogUartError(ctx, errorCode, 2);
    }
    if (errorCode & HAL_UART_ERROR_NE) {
        ctx->stats.noiseErrors++;
        ErrorHandler_LogUartError(ctx, errorCode, 3);
    }
    if (errorCode & HAL_UART_ERROR_ORE) {
        ctx->stats.overrunErrors++;
        ErrorHandler_LogUartError(ctx, errorCode, 4);
    }
    if (errorCode & HAL_UART_ERROR_DMA) {
        ctx->stats.dmaErrors++;
        ErrorHandler_LogUartError(ctx, errorCode, 5);
    }
    
    /* Check if hard reset is needed */
    if (ctx->stats.consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
        ctx->needsReinit = true;
        ctx->stats.reinitCount++;
    }
    
    /* Set exponential backoff for recovery */
    uint32_t delay = GetExponentialDelay(ERROR_RECOVERY_DELAY_MS, ctx->stats.recoveryAttempts);
    ctx->nextRecoveryTime = HAL_GetTick() + delay;
}

/**
  * @brief  Handle CAN error
  * @param  ctx: Pointer to CAN error context
  * @param  hcan: Pointer to CAN handle
  * @retval None
  */
void ErrorHandler_CanError(CanErrorContext_t *ctx, CAN_HandleTypeDef *hcan)
{
    if (ctx == NULL || hcan == NULL) return;
    
    uint32_t errorCode = hcan->ErrorCode;
    ctx->stats.totalErrors++;
    ctx->stats.consecutiveErrors++;
    ctx->stats.lastErrorTick = HAL_GetTick();
    
    /* Check CAN status register for detailed error info */
    uint32_t esr = hcan->Instance->ESR;
    uint8_t lec = (esr >> 4) & 0x07; /* Last Error Code */
    
    /* Classify errors based on LEC */
    switch (lec) {
        case 0x01: /* Stuff Error */
            ctx->stats.stuffErrors++;
            ErrorHandler_LogCanError(ctx, errorCode, 1);
            break;
        case 0x02: /* Form Error */
            ctx->stats.formErrors++;
            ErrorHandler_LogCanError(ctx, errorCode, 2);
            break;
        case 0x03: /* Acknowledgment Error */
            ctx->stats.ackErrors++;
            ErrorHandler_LogCanError(ctx, errorCode, 3);
            break;
        case 0x04: /* Bit 1 (recessive) Error */
            ctx->stats.bit1Errors++;
            ErrorHandler_LogCanError(ctx, errorCode, 4);
            break;
        case 0x05: /* Bit 0 (dominant) Error */
            ctx->stats.bit0Errors++;
            ErrorHandler_LogCanError(ctx, errorCode, 5);
            break;
        case 0x06: /* CRC Error */
            ctx->stats.crcErrors++;
            ErrorHandler_LogCanError(ctx, errorCode, 6);
            break;
        default:
            ErrorHandler_LogCanError(ctx, errorCode, 0);
            break;
    }
    
    /* Check for transmit errors */
    if (errorCode & HAL_CAN_ERROR_TX_TERR0 || 
        errorCode & HAL_CAN_ERROR_TX_TERR1 || 
        errorCode & HAL_CAN_ERROR_TX_TERR2) {
        ctx->stats.txErrors++;
    }
    
    /* Check for error warning/passive states */
    uint8_t tec = (esr >> 16) & 0xFF; /* Transmit Error Counter */
    uint8_t rec = (esr >> 24) & 0xFF; /* Receive Error Counter */
    
    if (tec >= 96 || rec >= 96) {
        ctx->stats.warningLevel++;
    }
    if (tec >= 128 || rec >= 128) {
        ctx->stats.passiveLevel++;
    }
    
    /* Check for Bus-Off state */
    if (errorCode & HAL_CAN_ERROR_BOF) {
        ctx->stats.busOffCount++;
        ctx->busOffDetected = true;
        ctx->needsReinit = true;
        ErrorHandler_LogCanError(ctx, errorCode, 7);
    }
    
    /* Check if reinitialization is needed */
    if (ctx->stats.consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
        ctx->needsReinit = true;
        ctx->stats.reinitCount++;
    }
    
    /* Set exponential backoff for recovery */
    uint32_t delay = GetExponentialDelay(ERROR_RECOVERY_DELAY_MS, ctx->stats.recoveryAttempts);
    ctx->nextRecoveryTime = HAL_GetTick() + delay;
}

/**
  * @brief  Mark successful UART operation
  * @param  ctx: Pointer to UART error context
  * @retval None
  */
void ErrorHandler_UartSuccess(UartErrorContext_t *ctx)
{
    if (ctx == NULL) return;
    
    ctx->stats.consecutiveErrors = 0;
    ctx->needsReinit = false;
}

/**
  * @brief  Mark successful CAN operation
  * @param  ctx: Pointer to CAN error context
  * @retval None
  */
void ErrorHandler_CanSuccess(CanErrorContext_t *ctx)
{
    if (ctx == NULL) return;
    
    ctx->stats.consecutiveErrors = 0;
    ctx->needsReinit = false;
    ctx->busOffDetected = false;
}

/**
  * @brief  Check if UART recovery should be attempted
  * @param  ctx: Pointer to UART error context
  * @retval true if recovery should be attempted
  */
bool ErrorHandler_ShouldRecoverUart(UartErrorContext_t *ctx)
{
    if (ctx == NULL) return false;
    
    uint32_t currentTick = HAL_GetTick();
    return (currentTick >= ctx->nextRecoveryTime);
}

/**
  * @brief  Check if CAN recovery should be attempted
  * @param  ctx: Pointer to CAN error context
  * @retval true if recovery should be attempted
  */
bool ErrorHandler_ShouldRecoverCan(CanErrorContext_t *ctx)
{
    if (ctx == NULL) return false;
    
    uint32_t currentTick = HAL_GetTick();
    return (currentTick >= ctx->nextRecoveryTime);
}

/**
  * @brief  Log UART error to circular buffer
  * @param  ctx: Pointer to UART error context
  * @param  errorCode: HAL error code
  * @param  errorType: Custom error type
  * @retval None
  */
void ErrorHandler_LogUartError(UartErrorContext_t *ctx, uint32_t errorCode, uint8_t errorType)
{
    if (ctx == NULL) return;
    
    LogError(ctx->log, &ctx->logIndex, errorCode, errorType);
}

/**
  * @brief  Log CAN error to circular buffer
  * @param  ctx: Pointer to CAN error context
  * @param  errorCode: HAL error code
  * @param  errorType: Custom error type
  * @retval None
  */
void ErrorHandler_LogCanError(CanErrorContext_t *ctx, uint32_t errorCode, uint8_t errorType)
{
    if (ctx == NULL) return;
    
    LogError(ctx->log, &ctx->logIndex, errorCode, errorType);
}

/**
  * @brief  Reinitialize UART peripheral
  * @param  huart: Pointer to UART handle
  * @retval HAL status
  */
HAL_StatusTypeDef ErrorHandler_ReinitUart(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status;
    
    /* Deinitialize UART */
    status = HAL_UART_DeInit(huart);
    if (status != HAL_OK) return status;
    
    /* Small delay before reinit */
    osDelay(REINIT_DELAY_MS / portTICK_PERIOD_MS);
    
    /* Reinitialize UART */
    status = HAL_UART_Init(huart);
    
    return status;
}

/**
  * @brief  Reinitialize CAN peripheral
  * @param  hcan: Pointer to CAN handle
  * @retval HAL status
  */
HAL_StatusTypeDef ErrorHandler_ReinitCan(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status;
    
    /* Stop CAN */
    status = HAL_CAN_Stop(hcan);
    if (status != HAL_OK) return status;
    
    /* Deinitialize CAN */
    status = HAL_CAN_DeInit(hcan);
    if (status != HAL_OK) return status;
    
    /* Small delay before reinit */
    osDelay(REINIT_DELAY_MS / portTICK_PERIOD_MS);
    
    /* Reinitialize CAN */
    status = HAL_CAN_Init(hcan);
    if (status != HAL_OK) return status;
    
    /* Reconfigure filter */
    CAN_FilterTypeDef canFilterConfig = {0};
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;
    
    status = HAL_CAN_ConfigFilter(hcan, &canFilterConfig);
    if (status != HAL_OK) return status;
    
    /* Restart CAN */
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK) return status;
    
    /* Reactivate notifications */
    status = HAL_CAN_ActivateNotification(hcan, 
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO1_MSG_PENDING |
        CAN_IT_TX_MAILBOX_EMPTY |
        CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF |
        CAN_IT_LAST_ERROR_CODE |
        CAN_IT_ERROR);
    
    return status;
}

/* Private functions */

/**
  * @brief  Log error to circular buffer
  * @param  log: Pointer to error log array
  * @param  index: Pointer to current index
  * @param  errorCode: Error code to log
  * @param  errorType: Error type classification
  * @retval None
  */
static void LogError(ErrorLogEntry_t *log, uint8_t *index, uint32_t errorCode, uint8_t errorType)
{
    if (log == NULL || index == NULL) return;
    
    log[*index].errorCode = errorCode;
    log[*index].timestamp = HAL_GetTick();
    log[*index].errorType = errorType;
    
    *index = (*index + 1) % ERROR_LOG_SIZE;
}

/**
  * @brief  Calculate exponential backoff delay
  * @param  baseDelay: Base delay in milliseconds
  * @param  attemptCount: Number of attempts
  * @retval Calculated delay with cap at 5 seconds
  */
static uint32_t GetExponentialDelay(uint32_t baseDelay, uint32_t attemptCount)
{
    uint32_t delay = baseDelay;
    
    for (uint32_t i = 0; i < attemptCount && i < 5; i++) {
        delay *= 2;
    }
    
    /* Cap at 5 seconds */
    if (delay > 5000) {
        delay = 5000;
    }
    
    return delay;
}
