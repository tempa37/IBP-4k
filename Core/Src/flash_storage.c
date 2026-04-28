#include "flash_storage.h"

#include "stm32f4xx_hal_flash.h"

/* Стирает непрерывный диапазон flash-секторов с единым unlock/lock. */
HAL_StatusTypeDef FLASH_EraseSectors(uint32_t firstSector, uint32_t lastSector)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0u;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = firstSector;
    eraseInit.NbSectors = (lastSector - firstSector + 1u);

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    return status;
}

/* Записывает буфер во flash побайтно, чтобы корректно принять неполные блоки. */
HAL_StatusTypeDef FLASH_WriteBuffer(uint32_t dstAddress, const uint8_t *src, uint32_t length)
{
    HAL_StatusTypeDef status = HAL_OK;

    HAL_FLASH_Unlock();

    for (uint32_t index = 0u; index < length; ++index)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, dstAddress + index, src[index]);
        if (status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    return status;
}

const uint16_t __attribute__((used)) lookup_table[256] = {
    0x0000, 0x0324, 0x0648, 0x096A, 0x0C8B, 0x0FAB, 0x12C8, 0x15E2,
    0x18F8, 0x1C0B, 0x1F19, 0x2223, 0x2528, 0x2826, 0x2B1F, 0x2E11,
    0x30FB, 0x33DE, 0x36BA, 0x398C, 0x3C56, 0x3F17, 0x41CE, 0x447A,
    0x471C, 0x49B4, 0x4C3F, 0x4EBF, 0x5133, 0x539B, 0x55F5, 0x5842,
    0x5A82, 0x5CB4, 0x5ED7, 0x60EC, 0x62F2, 0x64E8, 0x66CF, 0x68A6,
    0x6A6D, 0x6C24, 0x6DCA, 0x6F5F, 0x70E2, 0x7255, 0x73B5, 0x7504,
    0x7641, 0x776C, 0x7884, 0x798A, 0x7A7D, 0x7B5D, 0x7C29, 0x7CE3,
    0x7D8A, 0x7E1D, 0x7E9D, 0x7F09, 0x7F62, 0x7FA7, 0x7FD8, 0x7FF6
};

/* Считает CRC32 по данным, лежащим во flash по заданному адресу. */
uint32_t compute_flash_crc(uint32_t startAddress, uint32_t length)
{
    static uint32_t crc32_table[256] = {0};
    static uint8_t table_initialized = 0u;
    uint32_t crc = 0xFFFFFFFFUL;
    const uint8_t *data = (const uint8_t *)startAddress;

    if (table_initialized == 0u)
    {
        for (uint32_t index = 0u; index < 256u; ++index)
        {
            uint32_t table_crc = index << 24;

            for (uint8_t bit = 0u; bit < 8u; ++bit)
            {
                if ((table_crc & 0x80000000UL) != 0u)
                {
                    table_crc = (table_crc << 1) ^ 0x04C11DB7UL;
                }
                else
                {
                    table_crc <<= 1;
                }
            }

            crc32_table[index] = table_crc;
        }

        table_initialized = 1u;
    }

    while (length > 0u)
    {
        crc = crc32_table[(crc >> 24) ^ *data] ^ (crc << 8);
        ++data;
        --length;
    }

    return crc;
}
