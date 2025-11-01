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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WDI_Pin GPIO_PIN_0
#define WDI_GPIO_Port GPIOC
#define UART4_TX_Pin GPIO_PIN_0
#define UART4_TX_GPIO_Port GPIOA
#define BAT_4_Pin GPIO_PIN_13
#define BAT_4_GPIO_Port GPIOB
#define BAT_3_Pin GPIO_PIN_14
#define BAT_3_GPIO_Port GPIOB
#define BAT_2_Pin GPIO_PIN_15
#define BAT_2_GPIO_Port GPIOB
#define BAT_1_Pin GPIO_PIN_8
#define BAT_1_GPIO_Port GPIOD
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
