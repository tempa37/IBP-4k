#ifndef MODBUC_H
#define MODBUC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Адрес устройства Modbus согласно требованиям */
#define MODBUS_DEVICE_ADDRESS       (0x01u)

/* Количество опрашиваемых регистров Modbus */
#define MODBUS_REGISTER_COUNT       (37u)

/* Стартовый адрес регистров Modbus */
#define MODBUS_START_REGISTER       (0u)

/* Период опроса Modbus в миллисекундах */
#define MODBUS_POLL_PERIOD_MS       (1000u)

/* Максимальный размер Modbus-запроса на чтение регистров */
#define MODBUS_MAX_REQUEST_SIZE     (8u)

/* Вычисление ожидаемой длины ответа Modbus для заданного количества регистров */
size_t Modbus_GetExpectedResponseLength(uint16_t registerCount);

/* Формирование Modbus-запроса на чтение holding-регистров */
size_t Modbus_BuildReadHoldingRegistersRequest(uint8_t address,
                                               uint16_t startRegister,
                                               uint16_t registerCount,
                                               uint8_t *buffer,
                                               size_t bufferSize);

/* Разбор ответа Modbus и сохранение данных регистров */
bool Modbus_ParseHoldingRegistersResponse(const uint8_t *frame,
                                          size_t length,
                                          uint8_t expectedAddress,
                                          uint16_t registerCount,
                                          uint16_t *outRegisters);

#ifdef __cplusplus
}
#endif

#endif /* MODBUC_H */
