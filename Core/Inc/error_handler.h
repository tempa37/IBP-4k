/**
  ******************************************************************************
  * @file           : error_handler.h
  * @brief          : Enhanced error handling for UART and CAN peripherals
  ******************************************************************************
  * @attention
  *
  * Enhanced error tracking and recovery mechanisms for robust communication
  *
  ******************************************************************************
  */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* Error tracking configuration */
#define ERROR_LOG_SIZE              8u      /* Number of last errors to store */
#define MAX_CONSECUTIVE_ERRORS      5u      /* Threshold for hard reset */
#define ERROR_RECOVERY_DELAY_MS     100u    /* Base delay for recovery */
#define REINIT_DELAY_MS             500u    /* Delay before reinitialization */

/* UART error statistics */
typedef struct {
    uint32_t totalErrors;           /* Total error count */
    uint32_t consecutiveErrors;     /* Consecutive errors without success */
    uint32_t parityErrors;          /* HAL_UART_ERROR_PE */
    uint32_t framingErrors;         /* HAL_UART_ERROR_FE */
    uint32_t noiseErrors;           /* HAL_UART_ERROR_NE */
    uint32_t overrunErrors;         /* HAL_UART_ERROR_ORE */
    uint32_t dmaErrors;             /* HAL_UART_ERROR_DMA */
    uint32_t lastErrorTick;         /* Timestamp of last error */
    uint32_t recoveryAttempts;      /* Number of recovery attempts */
    uint32_t reinitCount;           /* Number of reinitialization cycles */
} UartErrorStats_t;

/* CAN error statistics */
typedef struct {
    uint32_t totalErrors;           /* Total error count */
    uint32_t consecutiveErrors;     /* Consecutive errors without success */
    uint32_t stuffErrors;           /* CAN_ESR bit errors */
    uint32_t formErrors;
    uint32_t ackErrors;
    uint32_t bit1Errors;
    uint32_t bit0Errors;
    uint32_t crcErrors;
    uint32_t txErrors;              /* Transmission errors */
    uint32_t warningLevel;          /* Error warning counter */
    uint32_t passiveLevel;          /* Error passive counter */
    uint32_t busOffCount;           /* Bus-off events */
    uint32_t lastErrorTick;         /* Timestamp of last error */
    uint32_t recoveryAttempts;      /* Number of recovery attempts */
    uint32_t reinitCount;           /* Number of reinitialization cycles */
} CanErrorStats_t;

/* Error log entry */
typedef struct {
    uint32_t errorCode;             /* HAL error code */
    uint32_t timestamp;             /* HAL_GetTick() value */
    uint8_t  errorType;             /* Custom error classification */
} ErrorLogEntry_t;

/* UART error context */
typedef struct {
    UartErrorStats_t stats;
    ErrorLogEntry_t log[ERROR_LOG_SIZE];
    uint8_t logIndex;               /* Circular buffer index */
    bool needsReinit;               /* Flag for reinitialization */
    uint32_t nextRecoveryTime;      /* Next allowed recovery attempt */
} UartErrorContext_t;

/* CAN error context */
typedef struct {
    CanErrorStats_t stats;
    ErrorLogEntry_t log[ERROR_LOG_SIZE];
    uint8_t logIndex;               /* Circular buffer index */
    bool needsReinit;               /* Flag for reinitialization */
    bool busOffDetected;            /* Bus-off state flag */
    uint32_t nextRecoveryTime;      /* Next allowed recovery attempt */
} CanErrorContext_t;

/* Function prototypes */
void ErrorHandler_InitUart(UartErrorContext_t *ctx);
void ErrorHandler_InitCan(CanErrorContext_t *ctx);

void ErrorHandler_UartError(UartErrorContext_t *ctx, UART_HandleTypeDef *huart);
void ErrorHandler_CanError(CanErrorContext_t *ctx, CAN_HandleTypeDef *hcan);

void ErrorHandler_UartSuccess(UartErrorContext_t *ctx);
void ErrorHandler_CanSuccess(CanErrorContext_t *ctx);

bool ErrorHandler_ShouldRecoverUart(UartErrorContext_t *ctx);
bool ErrorHandler_ShouldRecoverCan(CanErrorContext_t *ctx);

void ErrorHandler_LogUartError(UartErrorContext_t *ctx, uint32_t errorCode, uint8_t errorType);
void ErrorHandler_LogCanError(CanErrorContext_t *ctx, uint32_t errorCode, uint8_t errorType);

HAL_StatusTypeDef ErrorHandler_ReinitUart(UART_HandleTypeDef *huart);
HAL_StatusTypeDef ErrorHandler_ReinitCan(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_HANDLER_H */
