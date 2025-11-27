#ifndef CAN_H
#define CAN_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"

/* Макрос для записи регистра в CAN */
#define WRITE_REG_CAN(val) buffer_out[idx++] = (uint8_t)((val) >> 8); buffer_out[idx++] = (uint8_t)((val) & 0xFF);

/* Структура для хранения контекста CAN */
typedef struct
{
    osMutexId mutex;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    volatile bool dataReady;
    volatile bool errorDetected;
    volatile uint32_t errorCode;
} CanContext_t;

/* Структура для хранения всех данных от Modbus Slave */
typedef struct {
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
    uint16_t eeprom_addr_low;
    uint16_t error_flags;
    uint16_t firmware_version;
    int16_t  pack_current_raw_ma;
    int16_t  bq_curr_cal_x[2];
    int16_t  bq_curr_cal_y[2];
    int16_t  ina_curr_cal_x[2];
    int16_t  ina_curr_cal_y[2];
    uint16_t ina_volt_cal_x[2];
    uint16_t ina_volt_cal_y[2];
    bool     valid;
    uint32_t last_update_tick;
    uint8_t  slave_id;
} ModbusSlaveData_t;

/* Блоки данных батарейных модулей (чтение) */
#define CAN1_BLOCK_BASE     0x400U
#define CAN2_BLOCK_BASE     0x420U
#define CAN3_BLOCK_BASE     0x440U
#define CAN4_BLOCK_BASE     0x460U

/* Команды CAN */
#define CAN_CMD_GET_REGS    0x40FU
#define CAN_CMD_EN_PINS     0x40EU
#define CAN_WRITE_B1        0x4F1U
#define CAN_WRITE_B2        0x4F2U
#define CAN_WRITE_B3        0x4F3U
#define CAN_WRITE_B4        0x4F4U

/* Глобальные переменные CAN */
extern CanContext_t canContext;
extern uint8_t CanlocalData[8];

/* Функции CAN */
void CAN_Init(void);
void MX_CAN1_Init(void);
size_t SerializeModbusData(const ModbusSlaveData_t *data_in, uint8_t *buffer_out);
void SendBatteryDataToCAN(uint8_t block_index, CAN_HandleTypeDef *hcan);
bool CAN_HandleEnablePinsControl(const uint8_t *can_data, uint8_t data_length);

#endif /* CAN_H */