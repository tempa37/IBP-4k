#ifndef MODBUC_H
#define MODBUC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Максимальный размер Modbus-кадра, соответствующий количеству запрашиваемых регистров */
#define MODBUS_MAX_FRAME_SIZE            (128u)

/* Значение адреса ведомого устройства Modbus по умолчанию */
#define MODBUS_DEFAULT_SLAVE_ADDRESS     (0x01u)

/* Количество регистров, запрашиваемых от Modbus-устройства */
#define MODBUS_REGISTER_COUNT            (37u)

/* Структура для хранения последнего сформированного запроса и ответа */
typedef struct
{
  uint8_t request[MODBUS_MAX_FRAME_SIZE];   /* Буфер с последним отправленным запросом */
  size_t requestLength;                     /* Длина последнего запроса */
  uint8_t response[MODBUS_MAX_FRAME_SIZE];  /* Буфер с последним полученным ответом */
  size_t responseLength;                    /* Длина последнего ответа */
} ModbusChannelBuffer_t;

bool Modbus_PrepareReadRequest(ModbusChannelBuffer_t *buffer,
                               uint16_t startRegister,
                               uint16_t registerCount,
                               uint8_t slaveAddress);

bool Modbus_SaveResponse(ModbusChannelBuffer_t *buffer,
                         const uint8_t *data,
                         size_t length);

bool Modbus_DecodeRegisters(const ModbusChannelBuffer_t *buffer,
                            uint16_t *outRegisters,
                            size_t registerCount);

#ifdef __cplusplus
}
#endif

#endif /* MODBUC_H */
