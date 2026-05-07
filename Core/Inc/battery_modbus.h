#ifndef BATTERY_MODBUS_H
#define BATTERY_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Основная задача фонового UART/Modbus-опроса батарейных блоков. */
void BatteryModbus_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_MODBUS_H */
