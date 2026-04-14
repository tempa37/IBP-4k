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
#include "stm32f4xx_hal_flash.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modbuc.h"
#include "uart.h"
#include "can.h"
#include "slave_update_thread.h"
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
volatile uint32_t lastProcessedCanId = 0u;
volatile uint8_t lastProcessedCanDlc = 0u;
volatile uint32_t lastProcessedUartError = 0u;
volatile uint32_t lastProcessedCanError = 0u;
volatile uint8_t ips1_in = 0u;
volatile uint8_t ips2_in = 0u;
volatile uint8_t ips3_in = 0u;
volatile uint8_t ips4_in = 0u;

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

volatile uint32_t crc32 = 0;
volatile uint32_t iar_crc32 = 0;



/*
 * RAM-состояние одного сеанса приёма прошивки по CAN.
 *
 * Зачем структура нужна:
 * - CAN приносит образ не целиком, а маленькими пакетами по 6 байт;
 * - пакеты нужно собирать в блок и только потом писать блок во flash;
 * - надо помнить тип образа, ожидаемое число блоков и текущий прогресс.
 *
 * Принцип:
 * - на BEGIN структура инициализируется;
 * - на DATA блок накапливается в blockBuffer;
 * - после завершения данные о сохранённом образе переносятся в metadata во flash.
 */
typedef struct {
    uint8_t imageKind;                 // Тип образа: master/slave
    bool updateInProgress;              // Флаг: идет процесс обновления
    bool imageSizeExact;               // Размер передан точно, а не выведен из количества блоков
    uint16_t totalBlocksExpected;       // Общее количество блоков к записи
    uint16_t currentBlockNum;           // Номер текущего блока (0, 1, 2, ...)
    uint8_t packetsInCurrentBlock;      // Сколько пакетов уже получено в текущем блоке (0..9)
    uint8_t blockBuffer[FW_BLOCK_SIZE_BYTES]; // Буфер для накопления одного блока
    uint32_t writeAddress;              // Текущий адрес записи во flash
    uint32_t imageSizeBytes;           // Фактический размер принятого образа
    uint32_t bytesWritten;             // Сколько байт образа уже записано во flash
} FirmwareUpdateState_t;

#define FW_IMAGE_KIND_NONE      0u    /* В staging-слоте нет валидного образа. */
#define FW_IMAGE_KIND_MASTER    1u    /* В staging-слоте лежит master firmware IBP-4k. */
#define FW_IMAGE_KIND_SLAVE     2u    /* В staging-слоте лежит slave firmware. */

/*
 * Два шаблона состояния нужны только для разделения логики master/slave.
 * Физическая flash-область при этом одна общая, а активен в каждый момент
 * только один сеанс, на который указывает activeFwState.
 */
static FirmwareUpdateState_t slaveFwState = {0};
static FirmwareUpdateState_t masterFwState = {0};
static FirmwareUpdateState_t *activeFwState = NULL;

/*
 * RAM-копия persistent metadata из flash.
 *
 * Эти переменные удобны для отладки и для остального кода приложения.
 * Источник истины всё равно лежит во flash и на старте перечитывается заново.
 */
volatile uint32_t storedImageType = FW_IMAGE_KIND_NONE;
volatile uint32_t storedImageSize = 0u;
volatile uint32_t storedUpdateFlag = FLASH_UPDATE_FLAG_CLEAR_VALUE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
/* USER CODE BEGIN PFP */
void BAT_SetIndicator(uint8_t battery);
void BAT_UpdateAllIndicators(void);
bool ParseModbusResponse(const uint8_t *response, size_t length, ModbusSlaveData_t *data_out);
extern void Uart_CheckAndRecover(UartContext_t *ctx);


HAL_StatusTypeDef FLASH_EraseSectors(uint32_t firstSector, uint32_t lastSector); //стирает сектора
HAL_StatusTypeDef FLASH_WriteBuffer(uint32_t dstAddress, const uint8_t *src, uint32_t length);                            //записывает буффер во флеш 
static void FW_HandleDataPacket(FirmwareUpdateState_t *state,
                                uint8_t dstNode,
                                const uint8_t *canData,
                                uint8_t dataLength);  //Обработка кадра CAN с обновлением

static void FW_SendAck(uint8_t dstNode, const uint8_t *data, uint8_t dlc);
static bool FW_HandleExtendedUpdateCommand(uint32_t extId, const uint8_t *canData, uint8_t dataLength);
static bool FW_ParseBeginRequest(const uint8_t *canData,
                                 uint8_t dataLength,
                                 uint16_t *totalBlocks,
                                 uint32_t *imageSizeBytes,
                                 bool *imageSizeExact);
static bool FW_IsStoredImageInfoValid(uint8_t imageKind, uint32_t imageSizeBytes);
static uint16_t FW_CalculateExpectedBlocks(uint32_t imageSizeBytes);
static uint32_t FW_NormalizeImageSize(uint32_t imageSizeBytes);
static FirmwareUpdateState_t *FW_SelectStateByImageSize(uint32_t imageSizeBytes);
static uint32_t FW_GetStorageStartAddress(const FirmwareUpdateState_t *state);
static HAL_StatusTypeDef FW_EraseStorageForState(FirmwareUpdateState_t *state);
static uint32_t FW_EncodeStoredImageType(uint8_t imageKind);
static uint8_t FW_DecodeStoredImageType(uint32_t rawType);
static void FW_ApplyStoredImageInfo(uint8_t imageKind, uint32_t imageSizeBytes, uint32_t updateFlagValue);
static void FW_LoadStoredImageInfo(void);
static HAL_StatusTypeDef FW_WriteStoredImageInfo(uint8_t imageKind,
                                                 uint32_t imageSizeBytes,
                                                 uint32_t updateFlagValue);
static bool FW_VerifyStoredImageCrc(const FirmwareUpdateState_t *state,
                                    uint32_t *calculatedCrc,
                                    uint32_t *storedCrc);
uint32_t compute_flash_crc(uint32_t startAddress, uint32_t length);


HAL_StatusTypeDef Set_Update_Flag(uint32_t imageSizeBytes);
void CheckConnectionTimeout(void);





/*
typedef struct {
    volatile uint8_t ack;
    volatile uint8_t need_update;
} SlaveDevice_t;

typedef struct {
    SlaveDevice_t device[UART_CHANNEL_COUNT];
    volatile uint8_t os_in_flash;  
} SlaveSystem_t;
*/
SlaveSystem_t slave = {0};


//пример
//slave.device[0].ack = 0x79;       --ответ
//slave.device[0].need_update = 1;  --нужно обновление
//slave.os_in_flash = 1;            --ОС в памяти есть

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    // === Установка вектора прерываний приложения ===
    // SystemInit() может уже это делать, но лучше явно
    SCB->VTOR = FLASH_APP_START_ADDR;
    
    // Включить прерывания (если были выключены в bootloader)
    __enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  FW_LoadStoredImageInfo();

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1280); //1280
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 1280);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of WDI_Task */
  osThreadDef(WDI_Task, StartTask03, osPriorityIdle, 0, 128);
  WDI_TaskHandle = osThreadCreate(osThread(WDI_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(myTask04, StartTask04, osPriorityIdle, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask04), NULL);
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  //RCC_PLLP_DIV2 = 180
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

  /*Configure GPIO pins : IPS_4_Pin IPS_3_Pin IPS_2_Pin IPS_1_Pin */
  GPIO_InitStruct.Pin = IPS_4_Pin|IPS_3_Pin|IPS_2_Pin|IPS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
    if (!response || !data_out || length < 5u) return false;

    uint16_t expected_crc = Modbus_CalculateCRC(response, length - 2u);
    uint16_t received_crc = (uint16_t)response[length - 2u] |
                            ((uint16_t)response[length - 1u] << 8);

    if (expected_crc != received_crc) return false;

    // Проверка: Slave ID, Function Code, Byte Count
    uint8_t slave_id = response[0];
    uint8_t func     = response[1];
    uint8_t byte_cnt = response[2];

    if (func != 0x03 || byte_cnt != MODBUS_REGISTER_BYTE_COUNT) return false;
    if (length != ((size_t)byte_cnt + 5u)) return false;

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
    data_out->calibration_active_flag = READ_REG(21);
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
    data_out->nominal_capacity_mah = READ_REG(37);
    data_out->bq_coulomb_count_mah = READ_REG(38);

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
    
    volatile GPIO_PinState state = GPIO_PIN_SET;
    //volatile GPIO_PinState state = GPIO_PIN_RESET;
    
    //Если есть связь с блоком
    if(data->valid) 
    {
      state = GPIO_PIN_RESET;
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
    uint8_t dataLength = 0u;
    HAL_GPIO_WritePin(GPIOG,
                  EN_1_Pin | EN_2_Pin | EN_3_Pin | EN_4_Pin,
                  GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_ON2_GPIO_Port, RS485_ON2_Pin, GPIO_PIN_RESET);

  /* Infinite loop */
  for(;;)
  {
#if CAN_ENABLE == 1
    
    if (canContext.errorDetected)
    {
      taskENTER_CRITICAL();
      lastProcessedCanError = canContext.errorCode;
      lastProcessedCanId = 0u;
      lastProcessedCanDlc = 0u;
      canContext.errorDetected = false;
      canContext.errorCode = 0u;
      taskEXIT_CRITICAL();
    }

    while (CAN_DequeueReceivedFrame(&localHeader, CanlocalData, &dataLength))
    {
      if (dataLength < sizeof(CanlocalData))
      {
        memset(&CanlocalData[dataLength], 0, sizeof(CanlocalData) - dataLength);
      }
      lastProcessedCanId = (localHeader.IDE == CAN_ID_EXT) ? localHeader.ExtId : 0u;
      lastProcessedCanDlc = dataLength;

      if (localHeader.IDE == CAN_ID_EXT)
      {
        if (!FW_HandleExtendedUpdateCommand(localHeader.ExtId, CanlocalData, dataLength))
        {
          (void)CAN_HandleRegisterRequest(&hcan1, localHeader.ExtId);
        }
      }
    }
#endif
    
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

        
        /* Проверка на ACK/NACK от bootloader */
      if (length == 1u)
      {
        if (localBuffer[0] == 0x79)  // ACK
        {
          // Обработка ACK - успешное подтверждение команды
          // Здесь можно установить флаг для перехода к следующей команде bootloader
          slave.device[index].ack = localBuffer[0];
        }
        else if (localBuffer[0] == 0x1F)  // NACK
        {
          // Обработка NACK - команда отклонена
          // Можно повторить команду или обработать ошибку
          slave.device[index].ack = localBuffer[0];
        }
        else
        {
          // Неизвестный однобайтовый ответ
          modbusSlaveData[index].valid = false;
        }
      }
      else
      {
        
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
        BAT_UpdateAllIndicators();  // Обновление индикаторов (LED)
      }
      
      }
      
      /* Периодическая генерация Modbus-запросов к каждому из UART-устройств */
      if (!Slave_IsUpdateActive((uint8_t)index) &&
          ((currentTick - modbusLastPollTick[index]) >= modbusPollIntervalMs))
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
    
#endif

    /* Небольшая пауза, чтобы освободить процессор другим задачам */
    osDelay(10);
  }
  }
  /* USER CODE END StartTask02 */
}

uint32_t wdi_tick = 0;

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
  uint32_t lastIpsPollTick = 0u;

  ips1_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_1_GPIO_Port, IPS_1_Pin) == GPIO_PIN_SET);
  ips2_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_2_GPIO_Port, IPS_2_Pin) == GPIO_PIN_SET);
  ips3_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_3_GPIO_Port, IPS_3_Pin) == GPIO_PIN_SET);
  ips4_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_4_GPIO_Port, IPS_4_Pin) == GPIO_PIN_SET);
  lastIpsPollTick = HAL_GetTick();

  for(;;)
  {
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    osDelay(100);
    CheckConnectionTimeout(); //гасим LED
    wdi_tick++;

    uint32_t currentTick = HAL_GetTick();
    if ((currentTick - lastIpsPollTick) >= 1000u)
    {
      ips1_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_1_GPIO_Port, IPS_1_Pin) == GPIO_PIN_SET);
      ips2_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_2_GPIO_Port, IPS_2_Pin) == GPIO_PIN_SET);
      ips3_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_3_GPIO_Port, IPS_3_Pin) == GPIO_PIN_SET);
      ips4_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_4_GPIO_Port, IPS_4_Pin) == GPIO_PIN_SET);
      lastIpsPollTick = currentTick;
    }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask04 thread.
* @param 
* @retval 
*/
/* USER CODE END Header_StartTask02 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  


  for(;;)
  {
        // Основной обработчик процесса обновления slave
        Slave_UpdateProcess();
        
        // Задержка для других задач
        osDelay(50);
  }
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


/*
 * Стирание диапазона flash-секторов.
 *
 * Это низкоуровневый helper без знания о типе прошивки.
 * В логике обновления он используется как кирпичик для:
 * - очистки общей staging-области перед новым приёмом;
 * - полной перезаписи сектора metadata.
 *
 * Почему стирание вынесено отдельно:
 * - легче централизованно работать с unlock/lock flash;
 * - меньше риска, что разные ветки логики начнут стирать по-разному.
 */
HAL_StatusTypeDef FLASH_EraseSectors(uint32_t firstSector, uint32_t lastSector)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0;

    eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector       = firstSector;
    eraseInit.NbSectors    = (lastSector - firstSector + 1u);

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    return status;
}

/*
 * Побайтная запись буфера во flash.
 *
 * Почему именно побайтно, а не словами:
 * - транспорт отдаёт данные не обязательно выровненными под слово;
 * - последний блок часто неполный;
 * - здесь приоритет не скорость, а предсказуемая и простая запись ровно того
 *   количества байт, которое реально относится к образу.
 */
HAL_StatusTypeDef FLASH_WriteBuffer(uint32_t dstAddress,
                                           const uint8_t *src,
                                           uint32_t length)
{
    HAL_StatusTypeDef status = HAL_OK;

    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < length; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, dstAddress + i, src[i]);
        if (status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    return status;
}

/*
 * Разбор BEGIN-пакета обновления.
 *
 * Что делает:
 * - достаёт число блоков;
 * - если отправитель передал точный размер, забирает и его;
 * - если точного размера нет, грубо восстанавливает размер как blocks * 60.
 *
 * Почему это нужно:
 * - транспортный формат эволюционировал, поэтому поддерживаются оба варианта;
 * - вся дальнейшая логика выбора образа и проверки ожидаемого числа блоков
 *   завязана именно на размере.
 */
static bool FW_ParseBeginRequest(const uint8_t *canData,
                                 uint8_t dataLength,
                                 uint16_t *totalBlocks,
                                 uint32_t *imageSizeBytes,
                                 bool *imageSizeExact)
{
    uint16_t parsedBlocks = 0u;
    uint32_t parsedSize = 0u;
    bool parsedSizeIsExact = false;

    if ((canData == NULL) || (totalBlocks == NULL) || (imageSizeBytes == NULL) || (imageSizeExact == NULL))
    {
        return false;
    }

    if (dataLength < 2u)
    {
        return false;
    }

    parsedBlocks = (uint16_t)((canData[0] << 8) | canData[1]);

    if (dataLength >= 6u)
    {
        parsedSize = ((uint32_t)canData[2] << 24)
                   | ((uint32_t)canData[3] << 16)
                   | ((uint32_t)canData[4] << 8)
                   | (uint32_t)canData[5];
        parsedSizeIsExact = true;
    }
    else
    {
        parsedSize = (uint32_t)parsedBlocks * FW_BLOCK_SIZE_BYTES;
    }

    *totalBlocks = parsedBlocks;
    *imageSizeBytes = FW_NormalizeImageSize(parsedSize);
    *imageSizeExact = parsedSizeIsExact;

    return true;
}

/*
 * Проверка согласованности metadata.
 *
 * Здесь не происходит определения типа образа "с нуля". Функция только отвечает
 * на вопрос: можно ли доверять комбинации type + size, прочитанной из flash.
 *
 * Это защита от мусора после сбоя питания, битых записей и ручного ковыряния
 * памяти. Если комбинация выглядит подозрительно, код потом переводит состояние
 * в NONE, чтобы не жить в красивой, но лживой хуйне.
 */
static bool FW_IsStoredImageInfoValid(uint8_t imageKind, uint32_t imageSizeBytes)
{
    if (imageKind == FW_IMAGE_KIND_NONE)
    {
        return (imageSizeBytes == 0u);
    }

    if (imageKind == FW_IMAGE_KIND_MASTER)
    {
        return (imageSizeBytes == FLASH_APP_SIZE);
    }

    if (imageKind == FW_IMAGE_KIND_SLAVE)
    {
        return ((imageSizeBytes > 0u) &&
                (imageSizeBytes < FLASH_APP_SIZE) &&
                (imageSizeBytes <= FLASH_FW_STORAGE_SIZE));
    }

    return false;
}

/*
 * Перевод размера образа в ожидаемое число transport-блоков по 60 байт.
 *
 * Нужен как sanity-check: BEGIN не должен врать сам себе. Если отправитель
 * заявил размер и число блоков, а математика не сходится, приём не запускаем.
 */
static uint16_t FW_CalculateExpectedBlocks(uint32_t imageSizeBytes)
{
    if (imageSizeBytes == 0u)
    {
        return 0u;
    }

    return (uint16_t)((imageSizeBytes + FW_BLOCK_SIZE_BYTES - 1u) / FW_BLOCK_SIZE_BYTES);
}

/*
 * Нормализация размера образа.
 *
 * Зачем нужна:
 * - последний блок протокола всегда добивается паддингом до 60 байт;
 * - master firmware имеет жёсткий логический размер 64 KB;
 * - если отправитель прислал 64 KB + хвост паддинга в пределах одного блока,
 *   это всё ещё надо считать master firmware, а не "непонятно чем".
 */
static uint32_t FW_NormalizeImageSize(uint32_t imageSizeBytes)
{
    if ((imageSizeBytes >= FLASH_APP_SIZE) &&
        (imageSizeBytes <= (FLASH_APP_SIZE + FW_BLOCK_SIZE_BYTES - 1u)))
    {
        return FLASH_APP_SIZE;
    }

    return imageSizeBytes;
}

/*
 * Кодирование внутреннего enum типа образа в flash-сигнатуру.
 *
 * Во flash храним не просто 1/2, а читаемые сигнатуры MAST / SLAV.
 * Так проще смотреть память руками и проще диагностировать состояние в отладке.
 */
static uint32_t FW_EncodeStoredImageType(uint8_t imageKind)
{
    if (imageKind == FW_IMAGE_KIND_MASTER)
    {
        return FLASH_STORED_IMAGE_TYPE_MASTER_VALUE;
    }

    if (imageKind == FW_IMAGE_KIND_SLAVE)
    {
        return FLASH_STORED_IMAGE_TYPE_SLAVE_VALUE;
    }

    return FLASH_STORED_IMAGE_TYPE_NONE_VALUE;
}

/*
 * Обратное преобразование flash-сигнатуры в enum образа.
 *
 * Всё, что не похоже на известные MAST / SLAV, считаем NONE. Это специально
 * жёсткое поведение: неизвестное состояние лучше отбросить, чем тащить дальше.
 */
static uint8_t FW_DecodeStoredImageType(uint32_t rawType)
{
    if (rawType == FLASH_STORED_IMAGE_TYPE_MASTER_VALUE)
    {
        return FW_IMAGE_KIND_MASTER;
    }

    if (rawType == FLASH_STORED_IMAGE_TYPE_SLAVE_VALUE)
    {
        return FW_IMAGE_KIND_SLAVE;
    }

    return FW_IMAGE_KIND_NONE;
}

/*
 * Применение metadata к RAM-состоянию.
 *
 * Здесь нет записи во flash. Это только синхронизация оперативных переменных
 * после чтения metadata или после успешной записи metadata.
 *
 * Отдельно важно:
 * - если сохранён slave image, автоматически выставляются slave.os_in_flash
 *   и slave.os_size_bytes;
 * - если сохранён master image или ничего не сохранено, slave-флаги обнуляются.
 */
static void FW_ApplyStoredImageInfo(uint8_t imageKind,
                                    uint32_t imageSizeBytes,
                                    uint32_t updateFlagValue)
{
    storedImageType = imageKind;
    storedImageSize = imageSizeBytes;
    storedUpdateFlag = (updateFlagValue == FLASH_UPDATE_FLAG_SET_VALUE)
                     ? FLASH_UPDATE_FLAG_SET_VALUE
                     : FLASH_UPDATE_FLAG_CLEAR_VALUE;

    if ((imageKind == FW_IMAGE_KIND_SLAVE) && (imageSizeBytes > 0u))
    {
        slave.os_in_flash = 1u;
        slave.os_size_bytes = imageSizeBytes;
    }
    else
    {
        slave.os_in_flash = 0u;
        slave.os_size_bytes = 0u;
    }
}

/*
 * Загрузка metadata из flash на старте приложения.
 *
 * Принцип:
 * 1. читаем сырые слова из сектора metadata;
 * 2. декодируем тип;
 * 3. проверяем, что комбинация type + size валидна;
 * 4. переносим результат в RAM-переменные.
 *
 * Так приложение после reset понимает, что именно сейчас лежит в общем
 * staging-слоте, не пытаясь гадать по адресам или по остаткам старых флагов.
 */
static void FW_LoadStoredImageInfo(void)
{
    uint32_t rawUpdateFlag = *(const uint32_t *)FLASH_UPDATE_FLAG_ADDR;
    uint32_t rawImageType = *(const uint32_t *)FLASH_STORED_IMAGE_TYPE_ADDR;
    uint32_t rawImageSize = *(const uint32_t *)FLASH_STORED_IMAGE_SIZE_ADDR;
    uint8_t imageKind = FW_DecodeStoredImageType(rawImageType);
    uint32_t imageSizeBytes = rawImageSize;

    if (!FW_IsStoredImageInfoValid(imageKind, imageSizeBytes))
    {
        imageKind = FW_IMAGE_KIND_NONE;
        imageSizeBytes = 0u;
    }

    FW_ApplyStoredImageInfo(imageKind, imageSizeBytes, rawUpdateFlag);
}

/*
 * Полная перезапись metadata во flash.
 *
 * Почему функция стирает весь сектор:
 * - flash STM32 нельзя надёжно "дописать поверх" произвольного старого слова;
 * - metadata и update-flag живут в одном секторе;
 * - поэтому запись любого из этих полей всегда считается атомарным обновлением
 *   всей тройки: update flag + image type + image size.
 *
 * Что важно понимать:
 * - новый приём образа сначала очищает metadata до NONE;
 * - сохранение slave image пишет тип и размер, но оставляет update-flag пустым;
 * - сохранение master image после CRC пишет и тип, и размер, и update-flag.
 */
static HAL_StatusTypeDef FW_WriteStoredImageInfo(uint8_t imageKind,
                                                 uint32_t imageSizeBytes,
                                                 uint32_t updateFlagValue)
{
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t sectorError = 0u;
    uint32_t rawImageType = FW_EncodeStoredImageType(imageKind);

    if (!FW_IsStoredImageInfoValid(imageKind, imageSizeBytes))
    {
        return HAL_ERROR;
    }

    if ((updateFlagValue != FLASH_UPDATE_FLAG_SET_VALUE) &&
        (updateFlagValue != FLASH_UPDATE_FLAG_CLEAR_VALUE))
    {
        return HAL_ERROR;
    }

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = FLASH_UPDATE_FLAG_SECTOR;
    eraseInit.NbSectors = 1u;

    HAL_FLASH_Unlock();

    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    if (status == HAL_OK)
    {
        if (updateFlagValue != FLASH_UPDATE_FLAG_CLEAR_VALUE)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_UPDATE_FLAG_ADDR,
                                       updateFlagValue);
        }

        if ((status == HAL_OK) && (rawImageType != FLASH_STORED_IMAGE_TYPE_NONE_VALUE))
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_STORED_IMAGE_TYPE_ADDR,
                                       rawImageType);
        }

        if ((status == HAL_OK) && (imageSizeBytes != 0u))
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_STORED_IMAGE_SIZE_ADDR,
                                       imageSizeBytes);
        }
    }

    HAL_FLASH_Lock();

    if (status == HAL_OK)
    {
        FW_ApplyStoredImageInfo(imageKind, imageSizeBytes, updateFlagValue);
    }

    return status;
}

/*
 * Выбор логического типа принимаемого образа.
 *
 * На этапе приёма решение пока всё ещё принимается по размеру:
 * - ровно 64 KB => master firmware;
 * - меньше 64 KB => slave firmware.
 *
 * Почему здесь это всё ещё так:
 * - транспортный протокол пока не передаёт явный тип образа отдельным полем;
 * - message ID больше не используется как источник истины;
 * - после завершения приёма тип уже фиксируется в metadata как persistent-флаг.
 */
static FirmwareUpdateState_t *FW_SelectStateByImageSize(uint32_t imageSizeBytes)
{
    if (imageSizeBytes == FLASH_APP_SIZE)
    {
        return &masterFwState;
    }

    if ((imageSizeBytes > 0u) &&
        (imageSizeBytes < FLASH_APP_SIZE) &&
        (imageSizeBytes <= FLASH_FW_STORAGE_SIZE))
    {
        return &slaveFwState;
    }

    return NULL;
}

/*
 * Возврат физического адреса начала staging-слота.
 *
 * Несмотря на наличие двух логических состояний (master/slave), реальный слот
 * теперь один общий. Поэтому обе ветки возвращают один и тот же адрес.
 */
static uint32_t FW_GetStorageStartAddress(const FirmwareUpdateState_t *state)
{
    if ((state == &masterFwState) || (state == &slaveFwState))
    {
        return FLASH_FW_STORAGE_START_ADDR;
    }

    return 0u;
}

/*
 * Стирание общей staging-области.
 *
 * Логика простая: новый приём любого образа полностью вытесняет предыдущий.
 * Поэтому отдельного erase для master/slave больше нет, а стирается общий слот.
 */
static HAL_StatusTypeDef FW_EraseStorageForState(FirmwareUpdateState_t *state)
{
    if ((state == &masterFwState) || (state == &slaveFwState))
    {
        HAL_StatusTypeDef eraseStatus;

        taskENTER_CRITICAL();
        eraseStatus = FLASH_EraseSectors(FLASH_FW_STORAGE_START_SECTOR, FLASH_FW_STORAGE_END_SECTOR);
        taskEXIT_CRITICAL();
        return eraseStatus;
    }

    return HAL_ERROR;
}

/*
 * Проверка CRC сохранённого образа.
 *
 * Сейчас эта функция реально используется только для master firmware IBP-4k.
 * Для slave image CRC-проверка на этапе хранения отключена по текущему ТЗ.
 *
 * Принцип:
 * - считаем CRC по образу без последних 4 байт;
 * - последние 4 байта считаем сохранённым эталонным CRC;
 * - сравниваем значения и возвращаем результат.
 */
static bool FW_VerifyStoredImageCrc(const FirmwareUpdateState_t *state,
                                    uint32_t *calculatedCrc,
                                    uint32_t *storedCrc)
{
    uint32_t imageStart = 0u;
    uint32_t payloadLength = 0u;
    const uint8_t *crcPtr;

    if ((state == NULL) || (state->imageSizeBytes < sizeof(uint32_t)) ||
        (calculatedCrc == NULL) || (storedCrc == NULL))
    {
        return false;
    }

    imageStart = FW_GetStorageStartAddress(state);
    if (imageStart == 0u)
    {
        return false;
    }

    payloadLength = state->imageSizeBytes - sizeof(uint32_t);
    *calculatedCrc = compute_flash_crc(imageStart, payloadLength);

    crcPtr = (const uint8_t *)(imageStart + payloadLength);
    *storedCrc = ((uint32_t)crcPtr[0])
               | ((uint32_t)crcPtr[1] << 8)
               | ((uint32_t)crcPtr[2] << 16)
               | ((uint32_t)crcPtr[3] << 24);

    return (*calculatedCrc == *storedCrc);
}

/*
 * Отправка ACK по CAN для протокола обновления.
 *
 * ACK используется в двух местах:
 * - после BEGIN приёмник сообщает состояние стирания/готовности;
 * - после записи блока приёмник подтверждает номер записанного блока.
 *
 * Это отдельная функция, чтобы формат extended ACK не размазывался по коду.
 */
static void FW_SendAck(uint8_t dstNode, const uint8_t *data, uint8_t dlc)
{
    uint32_t ackId = CAN_BuildExtId(CAN_NODE_IBP_4K,
                                    dstNode,
                                    CAN_MSG_FW_ACK,
                                    CAN_PRIORITY_DEFAULT);

    CAN_SendExtendedFrame(&hcan1, ackId, data, dlc);
}

static bool FW_HandleExtendedUpdateCommand(uint32_t extId, const uint8_t *canData, uint8_t dataLength)
{
    uint8_t srcNode = 0u;
    uint8_t dstNode = 0u;
    uint16_t msgId = 0u;

    CAN_ParseExtId(extId, &srcNode, &dstNode, &msgId, NULL);

    if ((srcNode != CAN_NODE_KOU) || (dstNode != CAN_NODE_IBP_4K))
    {
        return false;
    }

    switch (msgId)
    {
        case CAN_MSG_FW_SLAVE_BEGIN:
        case CAN_MSG_FW_MASTER_BEGIN:
        {
            /*
             * BEGIN master/slave теперь обрабатываются одинаково.
             *
             * Принцип:
             * 1. читаем размер и число блоков;
             * 2. по размеру определяем, master это или slave;
             * 3. стираем общий staging-слот;
             * 4. очищаем metadata до NONE;
             * 5. запускаем новый активный сеанс приёма.
             *
             * Почему так:
             * - физическая область хранения одна;
             * - старый образ должен исчезнуть до начала нового приёма;
             * - metadata не должны врать, если новый приём оборвётся посередине.
             */
            uint16_t totalBlocks = 0u;
            uint32_t imageSizeBytes = 0u;
            bool imageSizeExact = false;
            uint8_t responseData[1] = {0u};
            HAL_StatusTypeDef eraseStatus = HAL_ERROR;
            HAL_StatusTypeDef metadataStatus = HAL_ERROR;
            FirmwareUpdateState_t *selectedState = NULL;

            if (Slave_IsAnyUpdateRunning())
            {
                FW_SendAck(srcNode, responseData, 1u);
                return true;
            }

            if (!FW_ParseBeginRequest(canData,
                                      dataLength,
                                      &totalBlocks,
                                      &imageSizeBytes,
                                      &imageSizeExact))
            {
                FW_SendAck(srcNode, responseData, 1u);
                return true;
            }

            selectedState = FW_SelectStateByImageSize(imageSizeBytes);
            if ((selectedState != NULL) &&
                (totalBlocks == FW_CalculateExpectedBlocks(imageSizeBytes)))
            {
                eraseStatus = FW_EraseStorageForState(selectedState);
                if (eraseStatus == HAL_OK)
                {
                    metadataStatus = FW_WriteStoredImageInfo(FW_IMAGE_KIND_NONE,
                                                             0u,
                                                             FLASH_UPDATE_FLAG_CLEAR_VALUE);
                }
            }

            if ((selectedState != NULL) &&
                (eraseStatus == HAL_OK) &&
                (metadataStatus == HAL_OK) &&
                (totalBlocks == FW_CalculateExpectedBlocks(imageSizeBytes)))
            {
                memset(&slaveFwState, 0, sizeof(slaveFwState));
                memset(&masterFwState, 0, sizeof(masterFwState));

                selectedState->imageKind = (selectedState == &masterFwState)
                                         ? FW_IMAGE_KIND_MASTER
                                         : FW_IMAGE_KIND_SLAVE;
                selectedState->updateInProgress = true;
                selectedState->imageSizeExact = imageSizeExact;
                selectedState->totalBlocksExpected = totalBlocks;
                selectedState->currentBlockNum = 0u;
                selectedState->packetsInCurrentBlock = 0u;
                selectedState->writeAddress = FW_GetStorageStartAddress(selectedState);
                selectedState->imageSizeBytes = imageSizeBytes;
                selectedState->bytesWritten = 0u;
                memset(selectedState->blockBuffer, 0xFF, sizeof(selectedState->blockBuffer));

                Slave_CancelPendingUpdates();
                activeFwState = selectedState;
                responseData[0] = 0xFFu;
            }
            else
            {
                activeFwState = NULL;
                if ((selectedState != NULL) &&
                    ((eraseStatus != HAL_OK) || (metadataStatus != HAL_OK)))
                {
                    CAN_ReportFlashWriteError();
                }
            }

            FW_SendAck(srcNode, responseData, 1u);
            return true;
        }

        case CAN_MSG_FW_SLAVE_DATA:
        case CAN_MSG_FW_MASTER_DATA:
            /*
             * DATA для master и slave тоже идут в один обработчик.
             * Источник истины здесь не message ID, а activeFwState, который был
             * выбран и инициализирован на этапе BEGIN.
             */
            FW_HandleDataPacket(activeFwState, srcNode, canData, dataLength);
            return true;

        case CAN_MSG_FW_SLAVE_UPDATE_START:
        {
            uint8_t slaveNum = 0xFFu;
            uint8_t responseData[2];
            bool requestAccepted = false;

            if (dataLength >= 1u)
            {
                slaveNum = canData[0];
            }

            if (SLAVE_FW_SEND_ENABLE != 0u)
            {
                requestAccepted = (slaveNum < UART_CHANNEL_COUNT)
                                ? Slave_RequestSingleUpdate(slaveNum)
                                : Slave_RequestConnectedUpdate();
            }

            responseData[0] = slaveNum;
            responseData[1] = requestAccepted ? 0xFFu : 0x00u;

            FW_SendAck(srcNode, responseData, 2u);
            return true;
        }

        default:
            return false;
    }
}

/*
 * Приём и запись payload-блоков прошивки.
 *
 * Это центральная транспортная функция обновления.
 *
 * Что она делает:
 * - принимает один CAN-пакет;
 * - кладёт его 6 байт в нужное место blockBuffer;
 * - когда блок собран, пишет его в staging flash;
 * - на последнем блоке завершает сеанс.
 *
 * Что важно по принципу:
 * - блоки считаются логическими кусками по 60 байт;
 * - физически master и slave пишутся в одну staging-область;
 * - различие между ними проявляется только на этапе завершения:
 *   master проверяет CRC и ставит update-flag,
 *   slave только фиксирует metadata о сохранённом образе.
 */
static void FW_HandleDataPacket(FirmwareUpdateState_t *state,
                                uint8_t dstNode,
                                const uint8_t *canData,
                                uint8_t dataLength)
{
    // Проверки корректности
    if (!state || !canData || dataLength < 3u)
    {
        return;  // Некорректные данные
    }

    if (!state->updateInProgress)
    {
        return;  // Обновление не инициализировано
    }

    // Извлекаем глобальный индекс пакета (первые 2 байта)
    uint16_t globalPacketIndex = (uint16_t)((canData[0] << 8) | canData[1]);

    // Данные прошивки начинаются с 3-го байта
    const uint8_t *payload = &canData[2];
    uint8_t payloadLen = (dataLength >= 2u) ? (dataLength - 2u) : 0u;

    // Ограничиваем размер данных
    if (payloadLen > FW_PACKET_DATA_SIZE)
    {
        payloadLen = FW_PACKET_DATA_SIZE;
    }

    // Вычисляем номер блока и позицию пакета в блоке из глобального индекса
    uint16_t calculatedBlockNum = globalPacketIndex / FW_BLOCK_SIZE_PACKETS;
    uint8_t packetNumInBlock = globalPacketIndex % FW_BLOCK_SIZE_PACKETS;

    // Проверка последовательности (опционально, можно закомментировать)
    // Если пришел пакет не из текущего блока - начинаем новый блок
    if (calculatedBlockNum != state->currentBlockNum)
    {
        // Сбрасываем текущий блок и переходим к новому
        state->currentBlockNum = calculatedBlockNum;
        state->packetsInCurrentBlock = 0u;
        memset(state->blockBuffer, 0xFF, sizeof(state->blockBuffer));  // Заполняем 0xFF на случай неполного блока
    }

    // Копируем данные пакета в буфер блока
    uint8_t offsetInBuffer = packetNumInBlock * FW_PACKET_DATA_SIZE;
    if ((offsetInBuffer + payloadLen) <= FW_BLOCK_SIZE_BYTES)
    {
        memcpy(&state->blockBuffer[offsetInBuffer], payload, payloadLen);
    }

    // Обновляем счетчик принятых пакетов
    if (packetNumInBlock >= state->packetsInCurrentBlock)
    {
        state->packetsInCurrentBlock = packetNumInBlock + 1u;
    }

    // Проверяем, собран ли блок полностью
    bool blockComplete = (state->packetsInCurrentBlock >= FW_BLOCK_SIZE_PACKETS);

    // Проверяем, это последний блок?

    // Если блок собран полностью ИЛИ это последний блок и пришли все пакеты
    if (blockComplete)
    {
        // Вычисляем количество байт для записи
        uint32_t remainingBytes = 0u;
        uint32_t bytesToWrite = 0u;
        uint8_t ackData[2];
        uint32_t calculatedCrc = 0u;
        uint32_t storedCrc = 0u;
        HAL_StatusTypeDef writeStatus;

        if (state->bytesWritten >= state->imageSizeBytes)
        {
            state->updateInProgress = false;
            activeFwState = NULL;
            return;
        }

        remainingBytes = state->imageSizeBytes - state->bytesWritten;
        bytesToWrite = (remainingBytes > FW_BLOCK_SIZE_BYTES) ? FW_BLOCK_SIZE_BYTES : remainingBytes;

        // Записываем блок во flash
        writeStatus = FLASH_WriteBuffer(state->writeAddress, 
                                        state->blockBuffer, 
                                        bytesToWrite);

        if (writeStatus != HAL_OK)
        {
            state->updateInProgress = false;
            activeFwState = NULL;
            CAN_ReportFlashWriteError();
            return;
        }

        // Формируем подтверждение: номер записанного блока
        state->bytesWritten += bytesToWrite;
        ackData[0] = (uint8_t)(state->currentBlockNum >> 8);
        ackData[1] = (uint8_t)(state->currentBlockNum & 0xFFu);

        // Отправляем extended ACK c номером записанного блока
        FW_SendAck(dstNode, ackData, 2u);

        // Переходим к следующему блоку
        state->currentBlockNum++;
        state->packetsInCurrentBlock = 0u;
        state->writeAddress += bytesToWrite;

        // Очищаем буфер для следующего блока
        memset(state->blockBuffer, 0xFF, sizeof(state->blockBuffer));

        /*
         * Если записали все блоки - завершаем обновление.
         *
         * Развилка намеренно разная:
         * - master: проверяем CRC, записываем metadata и ставим update-flag;
         * - slave: просто записываем metadata о том, что в общем слоте лежит
         *   slave image такого-то размера.
         */
        if (state->currentBlockNum >= state->totalBlocksExpected)
        {
            state->updateInProgress = false;
            activeFwState = NULL;

            if (state == &masterFwState)
            {
                crc32 = 0u;
                iar_crc32 = 0u;

                if (FW_VerifyStoredImageCrc(state, &calculatedCrc, &storedCrc))
                {
                    crc32 = calculatedCrc;
                    iar_crc32 = storedCrc;
                    if (Set_Update_Flag(state->imageSizeBytes) != HAL_OK)
                    {
                        CAN_ReportFlashWriteError();
                    }
                }
                else
                {
                    CAN_ReportFlashWriteError();
                }
            }
            else if (state == &slaveFwState)
            {
                if (FW_WriteStoredImageInfo(FW_IMAGE_KIND_SLAVE,
                                            state->imageSizeBytes,
                                            FLASH_UPDATE_FLAG_CLEAR_VALUE) != HAL_OK)
                {
                    CAN_ReportFlashWriteError();
                }
#if (SLAVE_AUTO_UPDATE_ENABLE != 0u)
                else
                {
                    (void)Slave_RequestAutoConnectedUpdate();
                }
#endif
            }
        }
    }
}
 

const uint16_t __attribute__((used)) lookup_table[256] = {
    0x0000, 0x0324, 0x0648, 0x096A, 0x0C8B, 0x0FAB, 0x12C8, 0x15E2,
    0x18F8, 0x1C0B, 0x1F19, 0x2223, 0x2528, 0x2826, 0x2B1F, 0x2E11,
    0x30FB, 0x33DE, 0x36BA, 0x398C, 0x3C56, 0x3F17, 0x41CE, 0x447A,
    0x471C, 0x49B4, 0x4C3F, 0x4EBF, 0x5133, 0x539B, 0x55F5, 0x5842,
    // ... повтори или сгенерируй 256 значений
    0x5A82, 0x5CB4, 0x5ED7, 0x60EC, 0x62F2, 0x64E8, 0x66CF, 0x68A6,
    0x6A6D, 0x6C24, 0x6DCA, 0x6F5F, 0x70E2, 0x7255, 0x73B5, 0x7504,
    0x7641, 0x776C, 0x7884, 0x798A, 0x7A7D, 0x7B5D, 0x7C29, 0x7CE3,
    0x7D8A, 0x7E1D, 0x7E9D, 0x7F09, 0x7F62, 0x7FA7, 0x7FD8, 0x7FF6
    // Дополни остальными значениями...
};



/*
 * Программный расчёт CRC32 по данным во flash.
 *
 * Почему функция общая:
 * - раньше была жёсткая привязка к старому слоту master firmware;
 * - теперь слот общий, поэтому функция принимает адрес и длину параметрами.
 *
 * На практике сейчас используется для master firmware.
 */
uint32_t compute_flash_crc(uint32_t startAddress, uint32_t length) 
{

    static uint32_t crc32_table[256] = {0};
    static uint8_t table_initialized = 0;

    if (!table_initialized) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t crc = i << 24;
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80000000UL) {
                    crc = (crc << 1) ^ 0x04C11DB7UL;
                } else {
                    crc <<= 1;
                }
            }
            crc32_table[i] = crc;
        }
        table_initialized = 1;
    }

    uint32_t crc = 0xFFFFFFFFUL;
    const uint8_t *data = (const uint8_t *)startAddress;

    while (length--) {
        crc = crc32_table[(crc >> 24) ^ *data++] ^ (crc << 8);
    }

    return crc;
}


/*
 * Фиксация готовности новой master firmware к применению.
 *
 * Что делает:
 * - пишет во flash metadata типа MAST и размер образа;
 * - ставит update-flag 0x1111;
 * - после успешной записи инициирует reset.
 *
 * Почему reset именно здесь:
 * - только после успешной записи metadata bootloader сможет понять,
 *   что в staging-слоте лежит валидная master firmware для применения.
 */
HAL_StatusTypeDef Set_Update_Flag(uint32_t imageSizeBytes)
{
    HAL_StatusTypeDef status = FW_WriteStoredImageInfo(FW_IMAGE_KIND_MASTER,
                                                       imageSizeBytes,
                                                       FLASH_UPDATE_FLAG_SET_VALUE);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_NVIC_SystemReset();
    return status;
}

/**
 * @brief Проверка таймаута соединения со слейвами.
 * Если данных не было более 4 секунд, сбрасывает флаг valid и гасит LED.
 */
void CheckConnectionTimeout(void)
{
    const uint32_t TIMEOUT_MS = 4000;
    uint32_t currentTick = HAL_GetTick();

    for (uint8_t i = 0; i < UART_CHANNEL_COUNT; i++)
    {
        // Работаем только если флаг сейчас установлен 
        if (modbusSlaveData[i].valid)
        
            if ((currentTick - modbusSlaveData[i].last_update_tick) > TIMEOUT_MS)
            {
                modbusSlaveData[i].valid = false; // Сбрасываем флаг "связь есть"
                BAT_SetIndicator(i);              // гасим индикатор для этого канала
            }
        }
  }


//------------------------------------------------------------------------------
