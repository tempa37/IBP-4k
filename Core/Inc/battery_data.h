#ifndef BATTERY_DATA_H
#define BATTERY_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

extern ModbusSlaveData_t modbusSlaveData[];

bool ParseModbusResponse(const uint8_t *response, size_t length, ModbusSlaveData_t *data_out);
void CheckConnectionTimeout(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_DATA_H */
