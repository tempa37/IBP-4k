#include "modbuc.h"

#include <string.h>

/* Вспомогательная функция вычисления CRC16 для протокола Modbus */
static uint16_t Modbus_CalculateCRC(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFu;
  if ((data == NULL) || (length == 0u))
  {
    return crc;
  }

  for (size_t index = 0; index < length; ++index)
  {
    crc ^= (uint16_t)data[index];
    for (uint8_t bit = 0; bit < 8u; ++bit)
    {
      if ((crc & 0x0001u) != 0u)
      {
        crc >>= 1u;
        crc ^= 0xA001u;
      }
      else
      {
        crc >>= 1u;
      }
    }
  }

  return crc;
}

/* Вычисление ожидаемой длины ответа Modbus в байтах */
size_t Modbus_GetExpectedResponseLength(uint16_t registerCount)
{
  return (size_t)(5u + ((size_t)registerCount * 2u));
}

/* Формирование Modbus-запроса на чтение holding-регистров */
size_t Modbus_BuildReadHoldingRegistersRequest(uint8_t address,
                                               uint16_t startRegister,
                                               uint16_t registerCount,
                                               uint8_t *buffer,
                                               size_t bufferSize)
{
  const size_t requestLength = 6u;
  const size_t fullLength = requestLength + 2u; /* CRC16 */

  if ((buffer == NULL) || (bufferSize < fullLength))
  {
    return 0u;
  }

  buffer[0] = address;
  buffer[1] = 0x03u; /* Функция чтения holding-регистров */
  buffer[2] = (uint8_t)((startRegister >> 8u) & 0xFFu);
  buffer[3] = (uint8_t)(startRegister & 0xFFu);
  buffer[4] = (uint8_t)((registerCount >> 8u) & 0xFFu);
  buffer[5] = (uint8_t)(registerCount & 0xFFu);

  const uint16_t crc = Modbus_CalculateCRC(buffer, requestLength);
  buffer[6] = (uint8_t)(crc & 0xFFu);
  buffer[7] = (uint8_t)((crc >> 8u) & 0xFFu);

  return fullLength;
}

/* Разбор ответа Modbus на чтение holding-регистров */
bool Modbus_ParseHoldingRegistersResponse(const uint8_t *frame,
                                          size_t length,
                                          uint8_t expectedAddress,
                                          uint16_t registerCount,
                                          uint16_t *outRegisters)
{
  if ((frame == NULL) || (outRegisters == NULL))
  {
    return false;
  }

  const size_t expectedLength = Modbus_GetExpectedResponseLength(registerCount);
  if (length < expectedLength)
  {
    return false;
  }

  const uint16_t receivedCrc = (uint16_t)frame[length - 2u] | ((uint16_t)frame[length - 1u] << 8u);
  const uint16_t calculatedCrc = Modbus_CalculateCRC(frame, length - 2u);
  if (receivedCrc != calculatedCrc)
  {
    return false;
  }

  if (frame[0] != expectedAddress)
  {
    return false;
  }

  if (frame[1] != 0x03u)
  {
    return false;
  }

  const uint8_t byteCount = frame[2];
  if (byteCount != (uint8_t)(registerCount * 2u))
  {
    return false;
  }

  for (uint16_t registerIndex = 0u; registerIndex < registerCount; ++registerIndex)
  {
    const size_t dataOffset = 3u + ((size_t)registerIndex * 2u);
    outRegisters[registerIndex] = ((uint16_t)frame[dataOffset] << 8u) | (uint16_t)frame[dataOffset + 1u];
  }

  return true;
}
