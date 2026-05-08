/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "uart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
  

/*
 * Параметры транспортного протокола обновления по CAN.
 *
 * Прошивка передаётся не произвольными кусками, а блоками фиксированного
 * размера. Один блок состоит из 10 CAN-пакетов, в каждом пакете полезных
 * данных 6 байт. Итого один flash-блок приёма = 60 байт.
 *
 * Почему это важно:
 * - по этим значениям считается ожидаемое количество блоков;
 * - ими же задаётся размер временного буфера накопления;
 * - паддинг последнего блока тоже завязан на эти константы.
 */
#define FW_BLOCK_SIZE_PACKETS       10u
#define FW_PACKET_DATA_SIZE         6u
#define FW_BLOCK_SIZE_BYTES         (FW_BLOCK_SIZE_PACKETS * FW_PACKET_DATA_SIZE)
  
  
  
/*
 * Область bootloader.
 *
 */
#define FLASH_BOOT_START_ADDR     0x08000000UL
#define FLASH_BOOT_END_ADDR       0x0803FFFFUL
#define FLASH_BOOT_SIZE           (FLASH_BOOT_END_ADDR - FLASH_BOOT_START_ADDR + 1)
#define FLASH_BOOT_START_SECTOR   FLASH_SECTOR_0
#define FLASH_BOOT_END_SECTOR     FLASH_SECTOR_5
#define FLASH_BOOT_SECTOR_COUNT   6

/*
 * Рабочая область приложения IBP-4k.
 *
 * - физически адрес попадает в сектор 6 размером 128 KB, но линковка и логика
 *   обновления считают допустимым только окно 64 KB.
 */
#define FLASH_APP_START_ADDR      0x08040000UL
#define FLASH_APP_END_ADDR        0x0804FFFFUL
#define FLASH_APP_SIZE            (FLASH_APP_END_ADDR - FLASH_APP_START_ADDR + 1)
#define FLASH_APP_START_SECTOR    FLASH_SECTOR_6
#define FLASH_APP_END_SECTOR      FLASH_SECTOR_6
#define FLASH_APP_SECTOR_COUNT    1

/*
 * Сейчас неиспользуемый зазор во flash.
 *
 * Сектор 12 частично используется
 * под metadata через отдельные адреса ниже, а остальной диапазон пока можно
 * считать резервом на будущее.
 */
#define FLASH_UNUSED_START_ADDR   0x080C0000UL
#define FLASH_UNUSED_END_ADDR     0x0810FFFFUL
#define FLASH_UNUSED_SIZE         (FLASH_UNUSED_END_ADDR - FLASH_UNUSED_START_ADDR + 1)

/*
 * Общая staging-область для принятого образа.
 *
 * Принцип работы такой:
 * - одновременно хранится только один образ;
 * - сюда кладётся либо новая прошивка IBP-4k, либо прошивка слейва;
 * - тип того, что лежит в этом слоте, определяется не по адресу, а по metadata;
 * - новый приём всегда стирает предыдущий образ в этом слоте.
 *
 */
#define FLASH_FW_STORAGE_START_ADDR      0x08110000UL
#define FLASH_FW_STORAGE_END_ADDR        0x0811FFFFUL
#define FLASH_FW_STORAGE_SIZE            (FLASH_FW_STORAGE_END_ADDR - FLASH_FW_STORAGE_START_ADDR + 1)
#define FLASH_FW_STORAGE_START_SECTOR    FLASH_SECTOR_16
#define FLASH_FW_STORAGE_END_SECTOR      FLASH_SECTOR_16
#define FLASH_FW_STORAGE_SECTOR_COUNT    1

/*
 * Резерв.
 *
 * Раньше здесь была отдельная staging-область под slave firmware.
 */
#define FLASH_RESERVED_START_ADDR        0x081A0000UL
#define FLASH_RESERVED_END_ADDR          0x081BFFFFUL
#define FLASH_RESERVED_SIZE              (FLASH_RESERVED_END_ADDR - FLASH_RESERVED_START_ADDR + 1)

/*
 * Metadata сохранённого образа.
 *
 * Здесь лежат три слова:
 * - update flag: нужен только для master-прошивки IBP-4k;
 * - image type: что именно лежит в общем staging-слоте;
 * - image size: размер образа в байтах.
 *
 * Принцип:
 * - при сохранении slave image update flag остаётся "пустым";
 * - при сохранении master image после успешного CRC сюда пишется и тип, и размер,
 *   и update flag;
 * - при старте приложения metadata читаются обратно и по ним восстанавливается
 *   RAM-состояние.
 */
#define FLASH_UPDATE_FLAG_ADDR           0x08100000UL
#define FLASH_STORED_IMAGE_TYPE_ADDR     (FLASH_UPDATE_FLAG_ADDR + 0x4u)
#define FLASH_STORED_IMAGE_SIZE_ADDR     (FLASH_UPDATE_FLAG_ADDR + 0x8u)
#define FLASH_UPDATE_FLAG_SECTOR         FLASH_SECTOR_12

/* Значение "master-образ готов к применению bootloader'ом". */
#define FLASH_UPDATE_FLAG_SET_VALUE      0x00001111UL
/* Значение стёртого слова.*/
#define FLASH_UPDATE_FLAG_CLEAR_VALUE    0xFFFFFFFFUL

/* Значение "в staging-слоте нет валидного образа". */
#define FLASH_STORED_IMAGE_TYPE_NONE_VALUE    0xFFFFFFFFUL
/* Четырёхсимвольная сигнатура MAST: в общем слоте лежит новая master firmware. */
#define FLASH_STORED_IMAGE_TYPE_MASTER_VALUE  0x4D415354UL
/* Четырёхсимвольная сигнатура SLAV: в общем слоте лежит firmware слейва. */
#define FLASH_STORED_IMAGE_TYPE_SLAVE_VALUE   0x534C4156UL
  
  
  
#define STM32_BOOTLOADER_ACK   0x79u
#define STM32_BOOTLOADER_NACK  0x1Fu
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile uint8_t ips1_in;
extern volatile uint8_t ips2_in;
extern volatile uint8_t ips3_in;
extern volatile uint8_t ips4_in;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WDI_Pin GPIO_PIN_0
#define WDI_GPIO_Port GPIOC
#define BAT_4_Pin GPIO_PIN_13
#define BAT_4_GPIO_Port GPIOB
#define BAT_3_Pin GPIO_PIN_14
#define BAT_3_GPIO_Port GPIOB
#define BAT_2_Pin GPIO_PIN_15
#define BAT_2_GPIO_Port GPIOB
#define BAT_1_Pin GPIO_PIN_8
#define BAT_1_GPIO_Port GPIOD
#define IPS_4_Pin GPIO_PIN_9
#define IPS_4_GPIO_Port GPIOD
#define IPS_3_Pin GPIO_PIN_10
#define IPS_3_GPIO_Port GPIOD
#define IPS_2_Pin GPIO_PIN_11
#define IPS_2_GPIO_Port GPIOD
#define IPS_1_Pin GPIO_PIN_12
#define IPS_1_GPIO_Port GPIOD
#define EN_4_Pin GPIO_PIN_3
#define EN_4_GPIO_Port GPIOG
#define EN_3_Pin GPIO_PIN_4
#define EN_3_GPIO_Port GPIOG
#define EN_2_Pin GPIO_PIN_5
#define EN_2_GPIO_Port GPIOG
#define EN_1_Pin GPIO_PIN_6
#define EN_1_GPIO_Port GPIOG
#define RS485_ON2_Pin GPIO_PIN_8
#define RS485_ON2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
