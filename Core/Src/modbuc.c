#include "modbuc.h"

#include <string.h>

/* Локальная функция для расчёта контрольной суммы Modbus CRC16 */
static uint16_t Modbus_CalculateCRC(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFu;

  if ((data == NULL) || (length == 0u))
  {
    return crc;
  }

  for (size_t index = 0u; index < length; ++index)
  {
    crc ^= data[index];
    for (uint8_t bit = 0u; bit < 8u; ++bit)
    {
      if ((crc & 0x0001u) != 0u)
      {
        crc >>= 1;
        crc ^= 0xA001u;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

bool Modbus_PrepareReadRequest(ModbusChannelBuffer_t *buffer,
                               uint16_t startRegister,
                               uint16_t registerCount,
                               uint8_t slaveAddress)
{
  if ((buffer == NULL) || (registerCount == 0u) || (registerCount > (MODBUS_MAX_FRAME_SIZE / 2u)))
  {
    return false;
  }

  /* Формируем стандартный запрос на чтение удерживаемых регистров (функция 0x03) */
  buffer->request[0] = slaveAddress;
  buffer->request[1] = 0x03u;
  buffer->request[2] = (uint8_t)(startRegister >> 8);
  buffer->request[3] = (uint8_t)(startRegister & 0xFFu);
  buffer->request[4] = (uint8_t)(registerCount >> 8);
  buffer->request[5] = (uint8_t)(registerCount & 0xFFu);

  uint16_t crc = Modbus_CalculateCRC(buffer->request, 6u);
  buffer->request[6] = (uint8_t)(crc & 0xFFu);
  buffer->request[7] = (uint8_t)((crc >> 8) & 0xFFu);
  buffer->requestLength = 8u;

  return true;
}

bool Modbus_SaveResponse(ModbusChannelBuffer_t *buffer,
                         const uint8_t *data,
                         size_t length)
{
  if ((buffer == NULL) || (data == NULL) || (length == 0u))
  {
    return false;
  }

  size_t copyLength = (length <= MODBUS_MAX_FRAME_SIZE) ? length : MODBUS_MAX_FRAME_SIZE;
  memcpy(buffer->response, data, copyLength);
  if (copyLength < MODBUS_MAX_FRAME_SIZE)
  {
    memset(&buffer->response[copyLength], 0, MODBUS_MAX_FRAME_SIZE - copyLength);
  }
  buffer->responseLength = copyLength;

  return true;
}

bool Modbus_DecodeRegisters(const ModbusChannelBuffer_t *buffer,
                            uint16_t *outRegisters,
                            size_t registerCount)
{
  if ((buffer == NULL) || (outRegisters == NULL) || (registerCount == 0u))
  {
    return false;
  }

  if (buffer->responseLength < 5u)
  {
    return false;
  }

  uint16_t expectedCrc = Modbus_CalculateCRC(buffer->response, buffer->responseLength - 2u);
  uint16_t receivedCrc = (uint16_t)buffer->response[buffer->responseLength - 2u] |
                         ((uint16_t)buffer->response[buffer->responseLength - 1u] << 8);
  if (expectedCrc != receivedCrc)
  {
    return false;
  }

  if (buffer->response[0] != buffer->request[0])
  {
    return false;
  }

  if (buffer->response[1] != 0x03u)
  {
    return false;
  }

  uint8_t byteCount = buffer->response[2];
  size_t minimalLength = (size_t)byteCount + 5u;
  if (buffer->responseLength < minimalLength)
  {
    return false;
  }

  size_t availableRegisters = byteCount / 2u;
  if (registerCount > availableRegisters)
  {
    registerCount = availableRegisters;
  }

  for (size_t index = 0u; index < registerCount; ++index)
  {
    size_t dataIndex = 3u + (index * 2u);
    outRegisters[index] = ((uint16_t)buffer->response[dataIndex] << 8) |
                          (uint16_t)buffer->response[dataIndex + 1u];
  }

  return true;
}
