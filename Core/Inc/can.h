#ifndef CAN_H
#define CAN_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"

#define CAN_RX_QUEUE_SIZE  32u

#define CAN_ERROR_LOG_FLAG_WARNING            (1UL << 0)
#define CAN_ERROR_LOG_FLAG_PASSIVE            (1UL << 1)
#define CAN_ERROR_LOG_FLAG_BUS_OFF            (1UL << 2)
#define CAN_ERROR_LOG_FLAG_STUFF              (1UL << 3)
#define CAN_ERROR_LOG_FLAG_FORM               (1UL << 4)
#define CAN_ERROR_LOG_FLAG_ACK                (1UL << 5)
#define CAN_ERROR_LOG_FLAG_BIT_RECESSIVE      (1UL << 6)
#define CAN_ERROR_LOG_FLAG_BIT_DOMINANT       (1UL << 7)
#define CAN_ERROR_LOG_FLAG_CRC                (1UL << 8)
#define CAN_ERROR_LOG_FLAG_RX_FIFO0_OVERRUN   (1UL << 9)
#define CAN_ERROR_LOG_FLAG_RX_FIFO1_OVERRUN   (1UL << 10)
#define CAN_ERROR_LOG_FLAG_TX_ARB_LOST        (1UL << 11)
#define CAN_ERROR_LOG_FLAG_TX_ERROR           (1UL << 12)
#define CAN_ERROR_LOG_FLAG_TIMEOUT            (1UL << 13)
#define CAN_ERROR_LOG_FLAG_HAL_STATE          (1UL << 14)
#define CAN_ERROR_LOG_FLAG_PARAM              (1UL << 15)
#define CAN_ERROR_LOG_FLAG_INTERNAL           (1UL << 16)
#define CAN_ERROR_LOG_FLAG_RX_QUEUE_OVERFLOW  (1UL << 17)
#define CAN_ERROR_LOG_FLAG_TX_MAILBOX_FULL    (1UL << 18)

typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CanRxFrame_t;

typedef struct
{
    osMutexId mutex;
    CanRxFrame_t rxQueue[CAN_RX_QUEUE_SIZE];
    volatile uint8_t rxHead;
    volatile uint8_t rxTail;
    volatile bool dataReady;
    volatile bool errorDetected;
    volatile uint32_t errorCode;
} CanContext_t;

typedef struct
{
    volatile uint32_t currentFlags;
    volatile uint32_t latchedFlags;
    volatile uint32_t lastHalError;
    volatile uint32_t eventCounter;
    volatile uint16_t busOffCounter;
    volatile uint16_t lastTimestampHours;
} CanErrorLog_t;

/* CAN node addresses from the specification */
#define CAN_NODE_KOU        1U
#define CAN_NODE_IBP_4K     20U

/* Request/response message identifiers from the specification */
#define CAN_MSG_INA260_VOLTAGE          20U
#define CAN_MSG_INA260_CURRENT          21U
#define CAN_MSG_BAT_SOC                 22U
#define CAN_MSG_BQ_STATUS               23U
#define CAN_MSG_ERROR_FLAGS             24U
#define CAN_MSG_UART_VERSION            25U
#define CAN_MSG_BAT_CAPACITY_MAH        26U
#define CAN_MSG_BAT_NOMINAL_CAP         27U
#define CAN_MSG_BQ_COULOMB_COUNT_MAH    28U
#define CAN_MSG_CAN_BUSOFF_COUNTER      30U
#define CAN_MSG_WDG_RESET_COUNTER       31U
#define CAN_MSG_LAST_ERROR_TIMESTAMP    32U
#define CAN_MSG_LAST_ERROR_CODE         33U

#define CAN_PRIORITY_DEFAULT            0U
#define CAN_PRIORITY_DIAGNOSTIC         3U

/* ERROR_FLAGS bits according to the specification */
#define CAN_ERROR_FLAG_BQ_ERROR             (1U << 0)
#define CAN_ERROR_FLAG_INA_ERROR            (1U << 1)
#define CAN_ERROR_FLAG_BALANCE_WARNING      (1U << 2)
#define CAN_ERROR_FLAG_BALANCE_CRITICAL     (1U << 3)
#define CAN_ERROR_FLAG_COMM_ERROR           (1U << 4)

#define CAN_BALANCE_WARNING_THRESHOLD_MV    50U
#define CAN_BALANCE_CRITICAL_THRESHOLD_MV   100U

/* Firmware update protocol uses extended CAN IDs addressed to CAN_NODE_IBP_4K. */
#define CAN_MSG_FW_SLAVE_BEGIN          40U
#define CAN_MSG_FW_SLAVE_DATA           41U
#define CAN_MSG_FW_ACK                  42U
#define CAN_MSG_FW_MASTER_BEGIN         43U
#define CAN_MSG_FW_MASTER_DATA          44U
#define CAN_MSG_FW_SLAVE_UPDATE_START   45U

extern CanContext_t canContext;
extern CanErrorLog_t canErrorLog;
extern uint8_t CanlocalData[8];

void MX_CAN1_Init(void);
uint32_t CAN_BuildExtId(uint8_t src, uint8_t dst, uint16_t msgId, uint8_t priority);
bool CAN_ParseExtId(uint32_t canId, uint8_t *src, uint8_t *dst, uint16_t *msgId, uint8_t *priority);
bool CAN_HandleRegisterRequest(CAN_HandleTypeDef *hcan, uint32_t extId);
bool CAN_DequeueReceivedFrame(CAN_RxHeaderTypeDef *header, uint8_t *data, uint8_t *dlc);
void CAN_SendExtendedFrame(CAN_HandleTypeDef *hcan,
                           uint32_t extId,
                           const uint8_t *data,
                           uint8_t dlc);
void CAN_ReportFlashWriteError(void);
void CAN_ProcessPendingDiagnosticLog(void);

#endif /* CAN_H */
