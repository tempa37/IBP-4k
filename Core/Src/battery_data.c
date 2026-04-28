#include "battery_data.h"

#include "board_io.h"
#include "main.h"
#include "modbuc.h"
#include "uart.h"

ModbusSlaveData_t modbusSlaveData[UART_CHANNEL_COUNT] = {0};

/* Парсит Modbus-ответ чтения регистров и обновляет структуру данных батареи. */
bool ParseModbusResponse(const uint8_t *response, size_t length, ModbusSlaveData_t *data_out)
{
    uint16_t expected_crc;
    uint16_t received_crc;
    uint8_t slave_id;
    uint8_t func;
    uint8_t byte_cnt;
    const uint8_t *payload;

    if ((response == NULL) || (data_out == NULL) || (length < 5u))
    {
        return false;
    }

    expected_crc = Modbus_CalculateCRC(response, length - 2u);
    received_crc = (uint16_t)response[length - 2u] |
                   ((uint16_t)response[length - 1u] << 8);
    if (expected_crc != received_crc)
    {
        return false;
    }

    slave_id = response[0];
    func = response[1];
    byte_cnt = response[2];
    if ((func != 0x03u) || (byte_cnt != MODBUS_REGISTER_BYTE_COUNT))
    {
        return false;
    }

    if (length != ((size_t)byte_cnt + 5u))
    {
        return false;
    }

    payload = &response[3];

#define BATTERY_READ_REG(idx) ((uint16_t)((uint16_t)payload[(idx) * 2u] << 8) | payload[((idx) * 2u) + 1u])

    data_out->slave_id = slave_id;
    data_out->valid = true;
    data_out->last_update_tick = HAL_GetTick();
    data_out->ina_config = BATTERY_READ_REG(0);
    data_out->voltage_mv = BATTERY_READ_REG(1);
    data_out->current_ma = (int16_t)BATTERY_READ_REG(2);
    data_out->power_mw = BATTERY_READ_REG(3);
    data_out->bq_status_state_sys = BATTERY_READ_REG(4);
    data_out->balancing_status = BATTERY_READ_REG(5);

    for (uint8_t index = 0u; index < 10u; ++index)
    {
        data_out->cell_voltage_mv[index] = BATTERY_READ_REG(6u + index);
    }

    data_out->pack_voltage_mv = BATTERY_READ_REG(16);
    data_out->pack_current_ma = (int16_t)BATTERY_READ_REG(17);
    data_out->ship_mode = BATTERY_READ_REG(18);
    data_out->capacity_mah = BATTERY_READ_REG(19);
    data_out->soc_percent = BATTERY_READ_REG(20);
    data_out->calibration_active_flag = BATTERY_READ_REG(21);
    data_out->error_flags = BATTERY_READ_REG(22);
    data_out->firmware_version = BATTERY_READ_REG(23);
    data_out->pack_current_raw_ma = (int16_t)BATTERY_READ_REG(24);
    data_out->bq_curr_cal_x[0] = (int16_t)BATTERY_READ_REG(25);
    data_out->bq_curr_cal_x[1] = (int16_t)BATTERY_READ_REG(26);
    data_out->bq_curr_cal_y[0] = (int16_t)BATTERY_READ_REG(27);
    data_out->bq_curr_cal_y[1] = (int16_t)BATTERY_READ_REG(28);
    data_out->ina_curr_cal_x[0] = (int16_t)BATTERY_READ_REG(29);
    data_out->ina_curr_cal_x[1] = (int16_t)BATTERY_READ_REG(30);
    data_out->ina_curr_cal_y[0] = (int16_t)BATTERY_READ_REG(31);
    data_out->ina_curr_cal_y[1] = (int16_t)BATTERY_READ_REG(32);
    data_out->ina_volt_cal_x[0] = BATTERY_READ_REG(33);
    data_out->ina_volt_cal_x[1] = BATTERY_READ_REG(34);
    data_out->ina_volt_cal_y[0] = BATTERY_READ_REG(35);
    data_out->ina_volt_cal_y[1] = BATTERY_READ_REG(36);
    data_out->nominal_capacity_mah = BATTERY_READ_REG(37);
    data_out->bq_coulomb_count_mah = BATTERY_READ_REG(38);

#undef BATTERY_READ_REG

    return true;
}

/* Сбрасывает признак связи с батарейным каналом, если давно не было валидных данных. */
void CheckConnectionTimeout(void)
{
    const uint32_t timeout_ms = 4000u;
    uint32_t currentTick = HAL_GetTick();

    for (uint8_t index = 0u; index < UART_CHANNEL_COUNT; ++index)
    {
        if (modbusSlaveData[index].valid &&
            ((currentTick - modbusSlaveData[index].last_update_tick) > timeout_ms))
        {
            modbusSlaveData[index].valid = false;
            BAT_SetIndicator(index);
        }
    }
}
