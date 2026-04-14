#ifndef CAN_H
#define CAN_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"

#define CAN_RX_QUEUE_SIZE  32u

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
    uint16_t ina_config;
    uint16_t voltage_mv;
    int16_t  current_ma;
    uint16_t power_mw;
    uint16_t bq_status_state_sys;
    uint16_t balancing_status;
    uint16_t cell_voltage_mv[10];
    uint16_t pack_voltage_mv;
    int16_t  pack_current_ma;
    uint16_t ship_mode;
    uint16_t capacity_mah;
    uint16_t soc_percent;
    uint16_t calibration_active_flag;
    uint16_t error_flags;
    uint16_t firmware_version;
    int16_t  pack_current_raw_ma;
    int16_t  bq_curr_cal_x[2];
    int16_t  bq_curr_cal_y[2];
    int16_t  ina_curr_cal_x[2];
    int16_t  ina_curr_cal_y[2];
    uint16_t ina_volt_cal_x[2];
    uint16_t ina_volt_cal_y[2];
    uint16_t nominal_capacity_mah;
    uint16_t bq_coulomb_count_mah;
    bool     valid;
    uint32_t last_update_tick;
    uint8_t  slave_id;
} ModbusSlaveData_t;

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

#endif /* CAN_H */
