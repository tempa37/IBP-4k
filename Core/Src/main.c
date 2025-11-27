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
#include "uart.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UARTS_ENABLE 1
#define CAN_ENABLE   1

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
/* Начальный регистр для чтения данных по Modbus */
static const uint16_t modbusStartRegister = 0u;

/* Переменные для отладки последних обработанных сообщений */
volatile uint8_t lastProcessedUartNumber = 0u;
volatile uint16_t lastProcessedCanId = 0u;
volatile uint8_t lastProcessedCanDlc = 0u;
volatile uint32_t lastProcessedUartError = 0u;
volatile uint32_t lastProcessedCanError = 0u;

/* Буферы для хранения Modbus-запросов и ответов по каждому UART */
static ModbusChannelBuffer_t modbusBuffers[UART_CHANNEL_COUNT] = {0};

/* Временные метки последнего опроса каждого UART */
static uint32_t modbusLastPollTick[UART_CHANNEL_COUNT] = {0u};

/* Определения мьютексов для буферов обмена */
osMutexDef(UART4BufferMutex);
osMutexDef(UART5BufferMutex);
osMutexDef(UART7BufferMutex);
osMutexDef(UART8BufferMutex);

/* Хранилище данных от Modbus Slave */
ModbusSlaveData_t modbusSlaveData[UART_CHANNEL_COUNT] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */
void BAT_SetIndicator(uint8_t battery);
void BAT_UpdateAllIndicators(void);
bool ParseModbusResponse(const uint8_t *response, size_t length, ModbusSlaveData_t *data_out);
extern int Modbus_WriteRegister(uint16_t reg_addr, uint16_t data, uint8_t battery_block);
extern void Uart_CheckAndRecover(UartContext_t *ctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  
#if UARTS_ENABLE == 1
  MX_UART8_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_UART7_Init();
#endif
  
#if CAN_ENABLE == 1
  MX_CAN1_Init();
#endif

  /* USER CODE BEGIN 2 */
  
  /* Инициализация контекстов UART */
  UART_Init();

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

  /* Запуск прерываемого приёма для всех UART-каналов */
  UART_StartAllReceptions();

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
}

/* USER CODE BEGIN 4 */

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
 * @brief Управление индикатором батареи (LED)
 */
void BAT_SetIndicator(uint8_t battery)
{
    if (battery >= UART_CHANNEL_COUNT) return;
    
    ModbusSlaveData_t *data = &modbusSlaveData[battery];
    
    GPIO_PinState state = GPIO_PIN_SET;
    
    
    //Если есть связь с блоком
    if(data->valid) 
    {
      GPIO_PinState state = GPIO_PIN_RESET;
    }

    
    
    switch (battery) {
        case 0:
            HAL_GPIO_WritePin(BAT_1_GPIO_Port, BAT_1_Pin, state);
            break;
        case 1:
            HAL_GPIO_WritePin(BAT_2_GPIO_Port, BAT_2_Pin, state);
            break;
        case 2:
            HAL_GPIO_WritePin(BAT_3_GPIO_Port, BAT_3_Pin, state);
            break;
        case 3:
            HAL_GPIO_WritePin(BAT_4_GPIO_Port, BAT_4_Pin, state);
            break;
    }
}

/**
 * @brief Обновление всех индикаторов батарей
 */
void BAT_UpdateAllIndicators(void)
{
    for (uint8_t i = 0; i < UART_CHANNEL_COUNT; i++) {
        BAT_SetIndicator(i);
    }
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
    HAL_GPIO_WritePin(GPIOG,
                  EN_1_Pin | EN_2_Pin | EN_3_Pin | EN_4_Pin,
                  GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_ON2_GPIO_Port, RS485_ON2_Pin, GPIO_PIN_RESET);
    
    // структура для передачи CAN 
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = 0x123;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    volatile uint8_t can_need_data = 0;
    uint8_t block_num = 0xFF;

  /* Infinite loop */
  for(;;)
  {
#if CAN_ENABLE == 1
    
    if (canContext.errorDetected)
    {
      if ((canContext.mutex != NULL) && (osMutexWait(canContext.mutex, osWaitForever) == osOK))
      {
        lastProcessedCanError = canContext.errorCode;
        lastProcessedCanId = 0u;
        lastProcessedCanDlc = 0u;
       
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
      memcpy(CanlocalData, canContext.data, sizeof(CanlocalData));
      uint8_t fillStart = (localHeader.DLC <= 8u) ? localHeader.DLC : 8u;
      if (fillStart < sizeof(CanlocalData))
      {
        memset(&CanlocalData[fillStart], 0, sizeof(CanlocalData) - fillStart);
      }
      canContext.dataReady = false;
      taskEXIT_CRITICAL();

      dataLength = (localHeader.DLC <= 8u) ? localHeader.DLC : 8u;
      if ((canContext.mutex != NULL) && (osMutexWait(canContext.mutex, osWaitForever) == osOK))
      {
        lastProcessedCanId = (localHeader.IDE == CAN_ID_EXT) ? localHeader.ExtId : localHeader.StdId;
        lastProcessedCanDlc = dataLength;
        
        block_num = 0xFF;
        
        switch (lastProcessedCanId)
        {
          case CAN_CMD_GET_REGS:
            can_need_data = 1;
            break;
            
          case CAN_WRITE_B1:
            block_num = 0;
            break;
            
          case CAN_WRITE_B2:
            block_num = 1;
            break;
            
          case CAN_WRITE_B3:
            block_num = 2;
            break;
            
          case CAN_WRITE_B4:
            block_num = 3;
            break;
            
          case CAN_CMD_EN_PINS:  // 0x40E - управление пинами EN/1-EN/4
            CAN_HandleEnablePinsControl(CanlocalData, dataLength);
            break;
        }
          
        if(block_num != 0xFF)
        {
          uint16_t reg_addr = CanlocalData[0];
          uint16_t reg_value = ((uint16_t)CanlocalData[1] << 8) | CanlocalData[2];
          // Записываем значение в регистр блока
          Modbus_WriteRegister(reg_addr, reg_value, block_num);
        }
        
        osMutexRelease(canContext.mutex);
      }
    }

    //-------------------------------------------------------------------------------------------------------------
    
    if(can_need_data)
    {
      for (uint8_t block = 0; block < UART_CHANNEL_COUNT; block++) 
      {
        SendBatteryDataToCAN(block, &hcan1);
      }
      can_need_data = 0;
    }
    
    osDelay(10); 

#endif
    
    osDelay(10);
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
  static const uint32_t modbusPollIntervalMs = 500u;
  /* Временный буфер для копирования принятого кадра */
  uint8_t localBuffer[UART_RX_CHUNK_SIZE] = {0};

  for(;;)
  {
    uint32_t currentTick = HAL_GetTick();

#if UARTS_ENABLE == 1
    
    
    
    for (size_t index = 0; index < UART_CHANNEL_COUNT; ++index)
    {
      UartContext_t *context = &uartContexts[index];
      Uart_CheckAndRecover(context);
      ModbusChannelBuffer_t *modbusBuffer = &modbusBuffers[index];

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
        
        /* Парсим ответ и заполняем структуру */
        if (ParseModbusResponse(localBuffer, length, &modbusSlaveData[index]))
        {
            // Данные успешно распаршены
        }
        else
        {
            // Ошибка парсинга
            modbusSlaveData[index].valid = false;
        }
        
        lastProcessedUartNumber = uartIndexToNumber[index];
      }

      BAT_UpdateAllIndicators();  // Обновление индикаторов (LED)
      
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
#endif

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
    osDelay(100);
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
