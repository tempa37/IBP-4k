#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

/* Глобальные переменные CAN */
CanContext_t canContext = {0};
uint8_t CanlocalData[8] = {0};

/* Внешние переменные */
extern CAN_HandleTypeDef hcan1;

/* Определение мьютекса для CAN */
osMutexDef(CANBufferMutex);

/**
 * @brief CAN1 Initialization Function
 */
void MX_CAN1_Init(void)
{
    __HAL_RCC_CAN1_FORCE_RESET();
    HAL_Delay(10);
    __HAL_RCC_CAN1_RELEASE_RESET();
    HAL_Delay(10);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 10;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
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
    
    if (HAL_CAN_ActivateNotification(&hcan1, 
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO1_MSG_PENDING |
        CAN_IT_TX_MAILBOX_EMPTY |
        CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF |
        CAN_IT_LAST_ERROR_CODE |
        CAN_IT_ERROR) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Обработчик готовности сообщения CAN из FIFO0
 */
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

/**
 * @brief Обработчик готовности сообщения CAN из FIFO1
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;
    
    CAN_RxHeaderTypeDef header = {0};
    uint8_t frameData[8] = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, frameData) != HAL_OK)
    {
        return;
    }
    // ✅ Сохраняем данные так же, как в FIFO0
    UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
    canContext.header = header;
    memcpy(canContext.data, frameData, sizeof(frameData));
    canContext.dataReady = true;
    taskEXIT_CRITICAL_FROM_ISR(irqState);
}

/**
 * @brief Обработчик ошибок CAN
 */
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
}

/**
 * @brief Сериализует структуру ModbusSlaveData_t в байтовый массив (big-endian)
 */
size_t SerializeModbusData(const ModbusSlaveData_t *data_in, uint8_t *buffer_out)
{
    if (!data_in || !buffer_out || !data_in->valid) return 0;
    
    size_t idx = 0;

    WRITE_REG_CAN(data_in->ina_config);
    WRITE_REG_CAN(data_in->voltage_mv);
    WRITE_REG_CAN((uint16_t)data_in->current_ma);
    WRITE_REG_CAN(data_in->power_mw);
    WRITE_REG_CAN(data_in->bq_status_state_sys);
    WRITE_REG_CAN(data_in->balancing_status);
    
    for (int i = 0; i < 10; i++) {
        WRITE_REG_CAN(data_in->cell_voltage_mv[i]);
    }
    
    WRITE_REG_CAN(data_in->pack_voltage_mv);
    WRITE_REG_CAN((uint16_t)data_in->pack_current_ma);
    WRITE_REG_CAN(data_in->ship_mode);
    WRITE_REG_CAN(data_in->capacity_mah);
    WRITE_REG_CAN(data_in->soc_percent);
    WRITE_REG_CAN(data_in->eeprom_addr_low);
    WRITE_REG_CAN(data_in->error_flags);
    WRITE_REG_CAN(data_in->firmware_version);
    WRITE_REG_CAN((uint16_t)data_in->pack_current_raw_ma);
    WRITE_REG_CAN((uint16_t)data_in->bq_curr_cal_x[0]);
    WRITE_REG_CAN((uint16_t)data_in->bq_curr_cal_x[1]);
    WRITE_REG_CAN((uint16_t)data_in->bq_curr_cal_y[0]);
    WRITE_REG_CAN((uint16_t)data_in->bq_curr_cal_y[1]);
    WRITE_REG_CAN((uint16_t)data_in->ina_curr_cal_x[0]);
    WRITE_REG_CAN((uint16_t)data_in->ina_curr_cal_x[1]);
    WRITE_REG_CAN((uint16_t)data_in->ina_curr_cal_y[0]);
    WRITE_REG_CAN((uint16_t)data_in->ina_curr_cal_y[1]);
    WRITE_REG_CAN(data_in->ina_volt_cal_x[0]);
    WRITE_REG_CAN(data_in->ina_volt_cal_x[1]);
    WRITE_REG_CAN(data_in->ina_volt_cal_y[0]);
    WRITE_REG_CAN(data_in->ina_volt_cal_y[1]);
    
    return idx;
}

/**
 * @brief Отправляет данные от одного батарейного блока по CAN (разбито на пакеты)
 */
void SendBatteryDataToCAN(uint8_t block_index, CAN_HandleTypeDef *hcan)
{
    extern ModbusSlaveData_t modbusSlaveData[];
    extern volatile uint32_t lastProcessedCanError;
    
    if (block_index >= 4 || !modbusSlaveData[block_index].valid) return;
    
    uint8_t serialized_buffer[74] = {0};
    size_t data_size = SerializeModbusData(&modbusSlaveData[block_index], serialized_buffer);
    if (data_size == 0) return;
    
    // Базовый ID для блока
    uint32_t base_id;
    switch (block_index) {
        case 0: base_id = CAN1_BLOCK_BASE; break;
        case 1: base_id = CAN2_BLOCK_BASE; break;
        case 2: base_id = CAN3_BLOCK_BASE; break;
        case 3: base_id = CAN4_BLOCK_BASE; break;
        default: return;
    }
    
    // Разбиение на пакеты (по 8 байт)
    size_t num_packets = (data_size + 7) / 8;
    for (size_t packet = 1; packet <= num_packets; packet++) {
        uint8_t packet_data[8] = {0};
        size_t offset = (packet - 1) * 8;
        size_t packet_len = (offset + 8 <= data_size) ? 8 : (data_size - offset);
        memcpy(packet_data, &serialized_buffer[offset], packet_len);
        
        CAN_TxHeaderTypeDef tx_header;
        tx_header.StdId = base_id + (packet - 1);
        tx_header.ExtId = 0;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.IDE = CAN_ID_STD;
        tx_header.DLC = packet_len;
        tx_header.TransmitGlobalTime = DISABLE;
        
        uint32_t tx_mailbox;
        if (osMutexWait(canContext.mutex, osWaitForever) == osOK) {
            if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
                HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &tx_header, packet_data, &tx_mailbox);
                if (status != HAL_OK) {
                    lastProcessedCanError = hcan->ErrorCode;
                }
            }
            osMutexRelease(canContext.mutex);
        }
        
        osDelay(10);
    }
}

/**
 * @brief Обработчик CAN-команды управления пинами EN/1 - EN/4
 */
bool CAN_HandleEnablePinsControl(const uint8_t *can_data, uint8_t data_length)
{
    if (can_data == NULL || data_length < 2)
    {
        return false;
    }
    
    uint8_t pin_number = can_data[0];
    uint8_t pin_state = can_data[1];
    
    if (pin_number < 1 || pin_number > 4)
    {
        return false;
    }
    
    if (pin_state > 1)
    {
        return false;
    }
    
    GPIO_PinState gpio_state = (pin_state == 0x00) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    
    uint16_t gpio_pin = 0;
    switch (pin_number)
    {
        case 0x01: gpio_pin = EN_1_Pin; break;
        case 0x02: gpio_pin = EN_2_Pin; break;
        case 0x03: gpio_pin = EN_3_Pin; break;
        case 0x04: gpio_pin = EN_4_Pin; break;
        default: return false;
    }
    
    HAL_GPIO_WritePin(GPIOG, gpio_pin, gpio_state);
    
    return true;
}




/* Отправка простого CAN-кадра с указанным CAN Id и пустыми/произвольными данными */
void CAN_SendSimpleFrame(CAN_HandleTypeDef *hcan,
                                uint32_t stdId,
                                const uint8_t *data,
                                uint8_t dlc)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t  txData[8] = {0};

    if (dlc > 8u) dlc = 8u;
    if (data != NULL && dlc > 0u)
    {
        memcpy(txData, data, dlc);
    }

    txHeader.StdId = stdId;
    txHeader.ExtId = 0;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.DLC   = dlc;
    txHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0u)
    {
        (void)HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);
    }
}