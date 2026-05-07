#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Стирает непрерывный диапазон flash-секторов. */
HAL_StatusTypeDef FLASH_EraseSectors(uint32_t firstSector, uint32_t lastSector);

/* Записывает буфер данных во flash по указанному адресу. */
HAL_StatusTypeDef FLASH_WriteBuffer(uint32_t dstAddress, const uint8_t *src, uint32_t length);

/* Считает CRC32 по данным, расположенным во flash. */
uint32_t compute_flash_crc(uint32_t startAddress, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORAGE_H */
