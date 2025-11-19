/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modbuc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_RX_CHUNK_SIZE              (MODBUS_MAX_FRAME_SIZE)  /* Размер блока приёма UART соответствует максимальному кадру Modbus */

/* Перечень используемых UART-каналов */
typedef enum
{  UART_CHANNEL_8 = 0u,
  UART_CHANNEL_7,
  UART_CHANNEL_4,
  UART_CHANNEL_5,
  UART_CHANNEL_COUNT
} UartChannel_t;

/* Структура для хранения контекста UART */
typedef struct
{
  UART_HandleTypeDef *handle;
  osMutexId mutex;
  uint8_t rxTransferBuffer[UART_RX_CHUNK_SIZE];
  uint8_t rxBuffer[UART_RX_CHUNK_SIZE];
  size_t rxLength;
  volatile bool dataReady;
  volatile bool errorDetected;
  volatile uint32_t errorCode;
} UartContext_t;

/* Структура для хранения контекста CAN */
typedef struct
{
  osMutexId mutex;
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  volatile bool dataReady;
  volatile bool errorDetected;
  volatile uint32_t errorCode;
} CanContext_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId WDI_TaskHandle;
/* USER CODE BEGIN PV */
/* Контексты UART с указателями на соответствующие периферийные модули */
static UartContext_t uartContexts[UART_CHANNEL_COUNT] =
{ 
  [UART_CHANNEL_8] = { .handle = &huart8 },
  [UART_CHANNEL_7] = { .handle = &huart7 },
  [UART_CHANNEL_4] = { .handle = &huart4 },
  [UART_CHANNEL_5] = { .handle = &huart5 }
};


/* Структура для хранения всех данных от Modbus Slave */
typedef struct {
    uint16_t ina_config;                    // 0
    uint16_t voltage_mv;                    // 1
    int16_t  current_ma;                    // 2
    uint16_t power_mw;                      // 3

    uint16_t bq_status_state_sys;           // 4
    uint16_t balancing_status;              // 5

    uint16_t cell_voltage_mv[10];           // 6–15

    uint16_t pack_voltage_mv;               // 16
    int16_t  pack_current_ma;               // 17
    uint16_t ship_mode;                     // 18
    uint16_t capacity_mah;                  // 19
    uint16_t soc_percent;                   // 20
    uint16_t eeprom_addr_low;               // 21
    uint16_t error_flags;                   // 22
    uint16_t firmware_version;              // 23
    int16_t  pack_current_raw_ma;           // 24

    // Калибровка BQ76930 (ток)
    int16_t  bq_curr_cal_x[2];              // 25–26
    int16_t  bq_curr_cal_y[2];              // 27–28

    // Калибровка INA260 (ток)
    int16_t  ina_curr_cal_x[2];             // 29–30
    int16_t  ina_curr_cal_y[2];             // 31–32

    // Калибровка INA260 (напряжение)
    uint16_t ina_volt_cal_x[2];             // 33–34
    uint16_t ina_volt_cal_y[2];             // 35–36

    // Дополнительно: флаги и метаданные
    bool     valid;                         // Данные валидны?
    uint32_t last_update_tick;              // Время последнего обновления
    uint8_t  slave_id;                      // ID слейва
} ModbusSlaveData_t;


/* Таблица соответствия индекса и номера UART для комментариев */
static const uint8_t uartIndexToNumber[UART_CHANNEL_COUNT] = {8u, 7u, 4u, 5u};

/* Контекст CAN для FIFO0 */
static CanContext_t canContext = {0};

/* Переменные для отладки последних обработанных сообщений */
static volatile uint8_t lastProcessedUartNumber = 0u;
static volatile uint32_t lastProcessedCanId = 0u;
static volatile uint8_t lastProcessedCanDlc = 0u;
static volatile uint32_t lastProcessedUartError = 0u;
static volatile uint32_t lastProcessedCanError = 0u;

/* Буферы для хранения Modbus-запросов и ответов по каждому UART */
static ModbusChannelBuffer_t modbusBuffers[UART_CHANNEL_COUNT] = {0};

/* Временные метки последнего опроса каждого UART */
static uint32_t modbusLastPollTick[UART_CHANNEL_COUNT] = {0u};

/* Определения мьютексов для буферов обмена */
osMutexDef(UART4BufferMutex);
osMutexDef(UART5BufferMutex);
osMutexDef(UART7BufferMutex);
osMutexDef(UART8BufferMutex);
osMutexDef(CANBufferMutex);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART8_Init(void);
static void MX_UART5_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);


/* USER CODE BEGIN PFP */
static UartContext_t *Uart_GetContext(UART_HandleTypeDef *handle);
static void Uart_StartReception(UartContext_t *context);



void BAT_SetIndicator(uint8_t battery);
void BAT_UpdateAllIndicators(void);
bool ParseModbusResponse(const uint8_t *response, size_t length, ModbusSlaveData_t *data_out);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ModbusSlaveData_t modbusSlaveData[UART_CHANNEL_COUNT] = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART8_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* Создание мьютексов для безопасного доступа к UART-буферам */
  uartContexts[UART_CHANNEL_4].mutex = osMutexCreate(osMutex(UART4BufferMutex));
  if (uartContexts[UART_CHANNEL_4].mutex == NULL)
  {
    Error_Handler();
  }
  uartContexts[UART_CHANNEL_5].mutex = osMutexCreate(osMutex(UART5BufferMutex));
  if (uartContexts[UART_CHANNEL_5].mutex == NULL)
  {
    Error_Handler();
  }
  uartContexts[UART_CHANNEL_7].mutex = osMutexCreate(osMutex(UART7BufferMutex));
  if (uartContexts[UART_CHANNEL_7].mutex == NULL)
  {
    Error_Handler();
  }
  uartContexts[UART_CHANNEL_8].mutex = osMutexCreate(osMutex(UART8BufferMutex));
  if (uartContexts[UART_CHANNEL_8].mutex == NULL)
  {
    Error_Handler();
  }

  /* Создание мьютекса для буфера CAN */
  canContext.mutex = osMutexCreate(osMutex(CANBufferMutex));
  if (canContext.mutex == NULL)
  {
    Error_Handler();
  }

  /* Настройка фильтра CAN и активация приёма */
  CAN_FilterTypeDef canFilterConfig = {0};
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* Запуск прерываемого приёма для всех UART-каналов */
  for (size_t index = 0; index < UART_CHANNEL_COUNT; ++index)
  {
    
    UartContext_t *ctx = &uartContexts[index];

    // Пропускаем UART4, если не инициализирован
    if (ctx->handle->Instance == NULL) {
      continue;  // ← пропустить UART4
    }
    
    Uart_StartReception(&uartContexts[index]);
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1280);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 1280);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of WDI_Task */
  osThreadDef(WDI_Task, StartTask03, osPriorityIdle, 0, 128);
  WDI_TaskHandle = osThreadCreate(osThread(WDI_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
  Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
   Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

//-=====================================================================================================================================================================================
/* Начальный регистр для чтения данных по Modbus */
static const uint16_t modbusStartRegister = 0u;
uint8_t swap_crc = 0;

typedef struct {
  uint32_t BaudRate;
  uint32_t WordLength;  // HAL: UART_WORDLENGTH_8B / _9B (или _7B где доступно)
  uint32_t StopBits;    // HAL: UART_STOPBITS_1 / _2
  uint32_t Parity;      // HAL: UART_PARITY_NONE / _EVEN / _ODD
} UartHwCfg;



const UartHwCfg myuart = {
  .BaudRate   = 115200u,
  .WordLength = UART_WORDLENGTH_9B,
  .StopBits   = UART_STOPBITS_1,
  .Parity     = UART_PARITY_EVEN
};




/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = myuart.BaudRate;
  huart4.Init.WordLength = myuart.WordLength;
  huart4.Init.StopBits = myuart.StopBits;
  huart4.Init.Parity = myuart.Parity;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = myuart.BaudRate;
  huart5.Init.WordLength = myuart.WordLength;
  huart5.Init.StopBits = myuart.StopBits;
  huart5.Init.Parity = myuart.Parity;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = myuart.BaudRate;
  huart7.Init.WordLength = myuart.WordLength;
  huart7.Init.StopBits = myuart.StopBits;
  huart7.Init.Parity = myuart.Parity;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = myuart.BaudRate;
  huart8.Init.WordLength = myuart.WordLength;
  huart8.Init.StopBits = myuart.StopBits;
  huart8.Init.Parity = myuart.Parity;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BAT_4_Pin|BAT_3_Pin|BAT_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT_1_GPIO_Port, BAT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EN_4_Pin|EN_3_Pin|EN_2_Pin|EN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_ON2_GPIO_Port, RS485_ON2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : WDI_Pin */
  GPIO_InitStruct.Pin = WDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_4_Pin BAT_3_Pin BAT_2_Pin */
  GPIO_InitStruct.Pin = BAT_4_Pin|BAT_3_Pin|BAT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_1_Pin */
  GPIO_InitStruct.Pin = BAT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_4_Pin EN_3_Pin EN_2_Pin EN_1_Pin */
  GPIO_InitStruct.Pin = EN_4_Pin|EN_3_Pin|EN_2_Pin|EN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_ON2_Pin */
  GPIO_InitStruct.Pin = RS485_ON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_ON2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Поиск контекста UART по указателю на хендл */
static UartContext_t *Uart_GetContext(UART_HandleTypeDef *handle)
{
  for (size_t index = 0; index < UART_CHANNEL_COUNT; ++index)
  {
    if (uartContexts[index].handle == handle)
    {
      return &uartContexts[index];
    }
  }
  return NULL;
}

/* Перезапуск приёма UART с использованием прерываний */
static void Uart_StartReception(UartContext_t *context)
{
  if (context == NULL)
  {
    return;
  }
  /* Запуск приёма UART до наступления тишины на линии */
  if (HAL_UARTEx_ReceiveToIdle_IT(context->handle, context->rxTransferBuffer, UART_RX_CHUNK_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Обработчик событий приёма UART с фиксацией тишины на линии */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  UartContext_t *context = Uart_GetContext(huart);
  if (context == NULL)
  {
    return;
  }

  UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
  size_t copyLength = (Size <= UART_RX_CHUNK_SIZE) ? Size : UART_RX_CHUNK_SIZE;
  memcpy(context->rxBuffer, context->rxTransferBuffer, copyLength);
  if (copyLength < UART_RX_CHUNK_SIZE)
  {
    memset(&context->rxBuffer[copyLength], 0, UART_RX_CHUNK_SIZE - copyLength);
  }
  context->rxLength = copyLength;
  context->dataReady = true;
  taskEXIT_CRITICAL_FROM_ISR(irqState);

  /* Комментарий: немедленно перезапускаем приём для следующего кадра */
  Uart_StartReception(context);
}

/* Совместимость с обработчиком полного буфера при работе без режима «тишина» */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UARTEx_RxEventCallback(huart, UART_RX_CHUNK_SIZE);
}

volatile uint8_t temp0 = 0;
volatile uint8_t uart12 = 0;

/* Обработчик ошибок UART */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartContext_t *context = Uart_GetContext(huart);
  if (context == NULL)
  {
    return;
  }
  
  
  UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
  
    

        
        
  if (huart->ErrorCode & HAL_UART_ERROR_PE) {
      temp0 = 1;
  }
  if (huart->ErrorCode & HAL_UART_ERROR_FE) {
      temp0 = 2;
  }
  if (huart->ErrorCode & HAL_UART_ERROR_NE) {
      temp0 = 3;
  }
  if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
      temp0 = 4;
  }
  if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
      temp0 = 5;
  }
  else 
  {
    temp0 = huart->ErrorCode;
  }
    
  
 
  
  // Определяем UART с помощью switch
  switch ((uint32_t)huart->Instance)
  {
    case (uint32_t)UART4:
      uart12 = 4;
      break;
    case (uint32_t)UART7:
      uart12 = 7;
      break;
    case (uint32_t)UART8:
      uart12 = 8;
      break;
    case (uint32_t)UART5:
      uart12 = 5;
      break;
    default:
      // неизвестный UART
      break;
  }
  

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart); 
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    
    
    uint32_t error_code = huart->ErrorCode;
    
    huart->Instance->DR; // Чтение DR для очистки некоторых флагов

    context->errorCode = huart->ErrorCode;
    context->errorDetected = true;
    context->dataReady = false;
    context->rxLength = 0u;
    taskEXIT_CRITICAL_FROM_ISR(irqState);
  
    
  /* Комментарий: перезапуск приёма UART после ошибки */
  Uart_StartReception(context);
}



/* Обработчик готовности сообщения CAN из FIFO0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
    return;
  }

  CAN_RxHeaderTypeDef header = {0};
  uint8_t frameData[8] = {0};
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frameData) != HAL_OK)
  {
    return;
  }

  UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
  canContext.header = header;
  memcpy(canContext.data, frameData, sizeof(frameData));
  canContext.dataReady = true;
  taskEXIT_CRITICAL_FROM_ISR(irqState);
}

/* Обработчик ошибок CAN */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
    return;
  }

  UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
  canContext.errorCode = hcan->ErrorCode;
  canContext.errorDetected = true;
  canContext.dataReady = false;
  memset(&canContext.header, 0, sizeof(canContext.header));
  memset(canContext.data, 0, sizeof(canContext.data));
  taskEXIT_CRITICAL_FROM_ISR(irqState);

  /* Комментарий: при необходимости можно выполнить дополнительный сброс CAN */
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    /* Локальные структуры для приёма CAN-кадров */
    CAN_RxHeaderTypeDef localHeader = {0};
    uint8_t localData[8] = {0};
    HAL_GPIO_WritePin(GPIOG,
                  EN_1_Pin | EN_2_Pin | EN_3_Pin | EN_4_Pin,
                  GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_ON2_GPIO_Port, RS485_ON2_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
    
    if (canContext.errorDetected)
    {
      if ((canContext.mutex != NULL) && (osMutexWait(canContext.mutex, osWaitForever) == osOK))
      {
        lastProcessedCanError = canContext.errorCode;
        lastProcessedCanId = 0u;
        lastProcessedCanDlc = 0u;
        /* Комментарий: ошибка CAN - lastProcessedCanError, номер шины - CAN1 */
        canContext.errorDetected = false;
        canContext.errorCode = 0u;
        osMutexRelease(canContext.mutex);
      }
    }
    else if (canContext.dataReady)
    {
      uint8_t dataLength = 0u;
      taskENTER_CRITICAL();
      localHeader = canContext.header;
      memcpy(localData, canContext.data, sizeof(localData));
      uint8_t fillStart = (localHeader.DLC <= 8u) ? localHeader.DLC : 8u;
      if (fillStart < sizeof(localData))
      {
        memset(&localData[fillStart], 0, sizeof(localData) - fillStart);
      }
      canContext.dataReady = false;
      taskEXIT_CRITICAL();

      dataLength = (localHeader.DLC <= 8u) ? localHeader.DLC : 8u;
      if ((canContext.mutex != NULL) && (osMutexWait(canContext.mutex, osWaitForever) == osOK))
      {
        lastProcessedCanId = (localHeader.IDE == CAN_ID_EXT) ? localHeader.ExtId : localHeader.StdId;
        lastProcessedCanDlc = dataLength;
        /* Комментарий: данные - localData, идентификатор CAN - lastProcessedCanId, DLC - dataLength */
        /* Здесь производится пользовательская обработка CAN-сообщения */
        osMutexRelease(canContext.mutex);
      }
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

  /* Период опроса всех устройств Modbus в миллисекундах */
  static const uint32_t modbusPollIntervalMs = 1000u;
  /* Временный буфер для копирования принятого кадра */
  uint8_t localBuffer[UART_RX_CHUNK_SIZE] = {0};


  
  for(;;)
  {
    
    uint32_t currentTick = HAL_GetTick();

    
    


    
    
    
    for (size_t index = 0; index < UART_CHANNEL_COUNT; ++index)
    {
      UartContext_t *context = &uartContexts[index];
      ModbusChannelBuffer_t *modbusBuffer = &modbusBuffers[index];

      /* Обработка ошибок UART с фиксацией последнего кода */
      if (context->errorDetected)
      {
        if ((context->mutex != NULL) && (osMutexWait(context->mutex, osWaitForever) == osOK))
        {
          uint8_t uartNumber = uartIndexToNumber[index];
          lastProcessedUartNumber = uartNumber;
          lastProcessedUartError = context->errorCode;
          /* Сбрасываем флаги ошибки после сохранения диагностической информации */
          context->errorDetected = false;
          context->errorCode = 0u;
          osMutexRelease(context->mutex);
        }
        continue;
      }

      /* Сохранение принятого Modbus-кадра в пользовательский буфер */
      if (context->dataReady)
      {
        size_t length = 0u;
        taskENTER_CRITICAL();
        length = context->rxLength;
        if (length > UART_RX_CHUNK_SIZE)
        {
          length = UART_RX_CHUNK_SIZE;
        }
        memcpy(localBuffer, context->rxBuffer, length);
        if (length < UART_RX_CHUNK_SIZE)
        {
          memset(&localBuffer[length], 0, UART_RX_CHUNK_SIZE - length);
        }
        context->dataReady = false;
        context->rxLength = 0u;
        taskEXIT_CRITICAL();

        (void)Modbus_SaveResponse(modbusBuffer, localBuffer, length);
        
        
        /* === НОВАЯ ФУНКЦИЯ: Парсим ответ и заполняем структуру === */
        if (ParseModbusResponse(localBuffer, length, &modbusSlaveData[index]))
        {
            // Данные успешно распаршены
            // Можно добавить логирование или событие
        }
        else
        {
            // Ошибка парсинга — можно сбросить валидность
            modbusSlaveData[index].valid = false;
        }
        
        
        
        lastProcessedUartNumber = uartIndexToNumber[index];
      }

      
      BAT_UpdateAllIndicators();  //-----Обновление индикаторов (LED)
      
      
      /* Периодическая генерация Modbus-запросов к каждому из UART-устройств */
      if ((currentTick - modbusLastPollTick[index]) >= modbusPollIntervalMs)
      {
        if ((context->mutex != NULL) && (osMutexWait(context->mutex, osWaitForever) == osOK))
        {
          if (Modbus_PrepareReadRequest(modbusBuffer, modbusStartRegister, MODBUS_REGISTER_COUNT, MODBUS_DEFAULT_SLAVE_ADDRESS))
          {
            HAL_StatusTypeDef st = HAL_UART_Transmit(context->handle, modbusBuffer->request, (uint16_t)modbusBuffer->requestLength, 1000u);
            if (st != HAL_OK) 
            {
              lastProcessedUartNumber = uartIndexToNumber[index];
              lastProcessedUartError  = uartContexts[index].handle->ErrorCode | (st << 24);
            }
          }
          osMutexRelease(context->mutex);
        }
        modbusLastPollTick[index] = currentTick;
      }
    }

    /* Небольшая пауза, чтобы освободить процессор другим задачам */
    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the WDI_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */


  for(;;)
  {
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/* USER CODE BEGIN 2 */

/**
 * @brief Парсит Modbus ответ (0x03) и заполняет структуру ModbusSlaveData_t
 * @param response: указатель на буфер с ответом (начинается с Slave ID)
 * @param length: длина буфера
 * @param data_out: указатель на структуру для записи
 * @return true если парсинг успешен, false — ошибка
 */
bool ParseModbusResponse(const uint8_t *response, size_t length, ModbusSlaveData_t *data_out)
{
    if (!response || !data_out || length < 5) return false;

    // Проверка: Slave ID, Function Code, Byte Count
    uint8_t slave_id = response[0];
    uint8_t func     = response[1];
    uint8_t byte_cnt = response[2];

    if (func != 0x03 || byte_cnt != 74) return false;
    //if (length < 3 + 128 + 2) return false;  // +2 CRC

    const uint8_t *payload = &response[3];  // Данные начинаются после byte count

    // Заполняем структуру
    data_out->slave_id = slave_id;
    data_out->valid = true;
    data_out->last_update_tick = HAL_GetTick();

    #define READ_REG(idx) ((uint16_t)(payload[(idx)*2] << 8) | payload[(idx)*2 + 1])

    data_out->ina_config           = READ_REG(0);
    data_out->voltage_mv           = READ_REG(1);
    data_out->current_ma           = (int16_t)READ_REG(2);
    data_out->power_mw             = READ_REG(3);

    data_out->bq_status_state_sys  = READ_REG(4);
    data_out->balancing_status     = READ_REG(5);

    for (int i = 0; i < 10; i++) {
        data_out->cell_voltage_mv[i] = READ_REG(6 + i);
    }

    data_out->pack_voltage_mv      = READ_REG(16);
    data_out->pack_current_ma      = (int16_t)READ_REG(17);
    data_out->ship_mode            = READ_REG(18);
    data_out->capacity_mah         = READ_REG(19);
    data_out->soc_percent          = READ_REG(20);
    data_out->eeprom_addr_low      = READ_REG(21);
    data_out->error_flags          = READ_REG(22);
    data_out->firmware_version     = READ_REG(23);
    data_out->pack_current_raw_ma  = (int16_t)READ_REG(24);

    data_out->bq_curr_cal_x[0]     = (int16_t)READ_REG(25);
    data_out->bq_curr_cal_x[1]     = (int16_t)READ_REG(26);
    data_out->bq_curr_cal_y[0]     = (int16_t)READ_REG(27);
    data_out->bq_curr_cal_y[1]     = (int16_t)READ_REG(28);

    data_out->ina_curr_cal_x[0]    = (int16_t)READ_REG(29);
    data_out->ina_curr_cal_x[1]    = (int16_t)READ_REG(30);
    data_out->ina_curr_cal_y[0]    = (int16_t)READ_REG(31);
    data_out->ina_curr_cal_y[1]    = (int16_t)READ_REG(32);

    data_out->ina_volt_cal_x[0]    = READ_REG(33);
    data_out->ina_volt_cal_x[1]    = READ_REG(34);
    data_out->ina_volt_cal_y[0]    = READ_REG(35);
    data_out->ina_volt_cal_y[1]    = READ_REG(36);

    #undef READ_REG

    return true;
}



/**
 * @brief Включает индикацию заданной батареи (0 = BAT_1, 1 = BAT_2, 2 = BAT_3, 3 = BAT_4)
 *        Все остальные индикаторы выключаются.
 * @param battery индекс батареи 0..3
 */
void BAT_SetIndicator(uint8_t battery)
{
    // Сначала выключаем все индикаторы
    //HAL_GPIO_WritePin(GPIOB, BAT_2_Pin | BAT_3_Pin | BAT_4_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(BAT_1_GPIO_Port, BAT_1_Pin, GPIO_PIN_RESET);

  

    // Затем включаем нужный
    switch (battery)
    {
        case 0:  // BAT_1
            HAL_GPIO_WritePin(BAT_1_GPIO_Port, BAT_1_Pin, GPIO_PIN_RESET);
            break;
        case 1:  // BAT_2
            HAL_GPIO_WritePin(GPIOB, BAT_2_Pin, GPIO_PIN_RESET);
            break;
        case 2:  // BAT_3
            HAL_GPIO_WritePin(GPIOB, BAT_3_Pin, GPIO_PIN_RESET);
            break;
        case 3:  // BAT_4
            HAL_GPIO_WritePin(GPIOB, BAT_4_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
  

}


/**
 * @brief Обновляет индикацию всех батарей согласно данным Modbus-слейвов
 */
void BAT_UpdateAllIndicators(void)
{
    // Сначала выключаем ВСЕ индикаторы
    HAL_GPIO_WritePin(BAT_1_GPIO_Port, BAT_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, BAT_2_Pin | BAT_3_Pin | BAT_4_Pin, GPIO_PIN_SET);


    // Проходим по всем слейвам (индексы 0..3)
    for (uint8_t i = 0; i < 4; i++)
    {
        if (modbusSlaveData[i].valid == 1)
        {
            BAT_SetIndicator(i);  // включаем нужный светодиод
        }
        // если valid == 0 — мы уже всё выключили выше, так что ничего не делаем
    }
}



/* USER CODE END 2 */
