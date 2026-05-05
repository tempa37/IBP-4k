#include "can.h"
#include "app_diagnostics.h"
#include "battery_data.h"
#include "diagnostic_log.h"
#include "FreeRTOS.h"
#include "task.h"

CanContext_t canContext = {0};
CanErrorLog_t canErrorLog = {0};
uint8_t CanlocalData[8] = {0};

extern CAN_HandleTypeDef hcan1;

static volatile uint16_t wdgResetCounter = 0u;
static volatile uint16_t lastErrorTimestampHours = 0u;
static volatile uint8_t lastErrorCode = 0u;

typedef struct
{
    volatile bool pending;
    volatile uint8_t errorCode;
    volatile uint8_t eventType;
    volatile uint8_t channel;
    volatile uint8_t source;
    volatile uint16_t timestampHours;
    volatile uint16_t canBusOffCounter;
    volatile uint16_t wdgCounter;
    volatile uint32_t flags;
    volatile uint32_t detail;
} CanPendingDiagnosticLog_t;

static CanPendingDiagnosticLog_t pendingDiagnosticLog = {0};

osMutexDef(CANBufferMutex);

static const uint16_t canRequestMsgIds[] =
{
    CAN_MSG_INA260_VOLTAGE,
    CAN_MSG_INA260_CURRENT,
    CAN_MSG_BAT_SOC,
    CAN_MSG_BQ_STATUS,
    CAN_MSG_ERROR_FLAGS,
    CAN_MSG_UART_VERSION,
    CAN_MSG_BAT_CAPACITY_MAH,
    CAN_MSG_BAT_NOMINAL_CAP,
    CAN_MSG_BQ_COULOMB_COUNT_MAH,
    CAN_MSG_CAN_BUSOFF_COUNTER,
    CAN_MSG_WDG_RESET_COUNTER,
    CAN_MSG_LAST_ERROR_TIMESTAMP,
    CAN_MSG_LAST_ERROR_CODE,
    CAN_MSG_DIAG_LOG_INFO,
    CAN_MSG_DIAG_LOG_READ
};

static const uint16_t canFirmwareMsgIds[] =
{
    CAN_MSG_FW_SLAVE_BEGIN,
    CAN_MSG_FW_SLAVE_DATA,
    CAN_MSG_FW_MASTER_BEGIN,
    CAN_MSG_FW_MASTER_DATA,
    CAN_MSG_FW_SLAVE_UPDATE_START
};

static uint32_t CAN_FilterEncodeExtendedId(uint32_t extId)
{
    return (extId << 3) | CAN_ID_EXT;
}

static HAL_StatusTypeDef CAN_ConfigFilterBank(uint8_t bank, uint32_t encodedId1, uint32_t encodedId2)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = bank;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (encodedId1 >> 16) & 0xFFFFu;
    filter.FilterIdLow = encodedId1 & 0xFFFFu;
    filter.FilterMaskIdHigh = (encodedId2 >> 16) & 0xFFFFu;
    filter.FilterMaskIdLow = encodedId2 & 0xFFFFu;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(&hcan1, &filter);
}

static HAL_StatusTypeDef CAN_ConfigHardwareFilters(void)
{
    volatile uint32_t fr1 = 0;
    volatile uint32_t fr2 = 0;
    
    
    uint32_t encodedIds[(sizeof(canRequestMsgIds) / sizeof(canRequestMsgIds[0])) +
                        (sizeof(canFirmwareMsgIds) / sizeof(canFirmwareMsgIds[0]))] = {0};
    size_t encodedCount = 0u;

    for (size_t index = 0u; index < (sizeof(canRequestMsgIds) / sizeof(canRequestMsgIds[0])); ++index)
    {
        encodedIds[encodedCount++] = CAN_FilterEncodeExtendedId(
            CAN_BuildExtId(CAN_NODE_KOU, CAN_NODE_IBP_4K, canRequestMsgIds[index], CAN_PRIORITY_DEFAULT));
    }

    for (size_t index = 0u; index < (sizeof(canFirmwareMsgIds) / sizeof(canFirmwareMsgIds[0])); ++index)
    {
        encodedIds[encodedCount++] = CAN_FilterEncodeExtendedId(
            CAN_BuildExtId(CAN_NODE_KOU, CAN_NODE_IBP_4K, canFirmwareMsgIds[index], CAN_PRIORITY_DEFAULT));
    }

    for (size_t index = 0u, bank = 0u; index < encodedCount; index += 2u, ++bank)
    {
        uint32_t encodedId1 = encodedIds[index];
        uint32_t encodedId2 = (index + 1u < encodedCount) ? encodedIds[index + 1u] : encodedIds[index];

        if (CAN_ConfigFilterBank((uint8_t)bank, encodedId1, encodedId2) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    
    
    fr1 = CAN1->sFilterRegister[6].FR1;
    fr2 = CAN1->sFilterRegister[6].FR2;
      
      
    return HAL_OK;
}

static void CAN_RecordErrorCode(uint8_t errorCodeValue);
static void CAN_LoadPersistentDiagnostics(void);
static HAL_StatusTypeDef CAN_WriteDiagnosticLogEvent(uint8_t errorCode,
                                                     uint8_t eventType,
                                                     uint8_t channel,
                                                     uint8_t source,
                                                     uint32_t flags,
                                                     uint32_t detail);
static HAL_StatusTypeDef CAN_WriteDiagnosticLogEventSnapshot(uint8_t errorCode,
                                                             uint8_t eventType,
                                                             uint8_t channel,
                                                             uint8_t source,
                                                             uint32_t flags,
                                                             uint32_t detail,
                                                             uint16_t timestampHours,
                                                             uint16_t canBusOffCounter,
                                                             uint16_t watchdogCounter);
static void CAN_CheckLastDiagnosticLogRecord(uint8_t sourceErrorCode);
static void CAN_QueueDiagnosticLogEvent(uint8_t errorCode,
                                        uint8_t eventType,
                                        uint8_t channel,
                                        uint8_t source,
                                        uint32_t flags,
                                        uint32_t detail);
static uint32_t CAN_BuildErrorLogFlags(uint32_t halError);
static void CAN_RecordErrorLog(uint32_t halError, uint32_t flags);
static void CAN_QueueRxFrameFromIsr(const CAN_RxHeaderTypeDef *header, const uint8_t *frameData);

static uint32_t CAN_GetStartupResetFlags(void)
{
    uint32_t resetFlags = 0u;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_BOR;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_PIN;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_POR;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_SOFTWARE;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_IWDG;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_WWDG;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET)
    {
        resetFlags |= DIAG_LOG_RESET_FLAG_LOW_POWER;
    }

    return resetFlags;
}

static void CAN_RecordStartupResetFlags(void)
{
    uint32_t resetFlags = CAN_GetStartupResetFlags();
    uint32_t resetDetail = RCC->CSR;
    uint8_t resetErrorCode = DIAG_LOG_ERROR_STARTUP_RESET;

    if (resetFlags == 0u)
    {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        return;
    }

    if ((resetFlags & (DIAG_LOG_RESET_FLAG_IWDG | DIAG_LOG_RESET_FLAG_WWDG)) != 0u)
    {
        resetErrorCode = DIAG_LOG_ERROR_WDG_RESET;

        if (wdgResetCounter < 0xFFFFu)
        {
            ++wdgResetCounter;
        }
    }

    CAN_RecordErrorCode(resetErrorCode);
    (void)CAN_WriteDiagnosticLogEvent(resetErrorCode,
                                      DIAG_LOG_EVENT_ERROR,
                                      DIAG_LOG_CHANNEL_GLOBAL,
                                      DIAG_LOG_SOURCE_RESET,
                                      resetFlags,
                                      resetDetail);

    __HAL_RCC_CLEAR_RESET_FLAGS();
}

static void CAN_RecordErrorCode(uint8_t errorCodeValue)
{
    lastErrorCode = errorCodeValue;
    lastErrorTimestampHours = (uint16_t)(HAL_GetTick() / 3600000u);
}

static HAL_StatusTypeDef CAN_WriteDiagnosticLogEvent(uint8_t errorCode,
                                                     uint8_t eventType,
                                                     uint8_t channel,
                                                     uint8_t source,
                                                     uint32_t flags,
                                                     uint32_t detail)
{
    return CAN_WriteDiagnosticLogEventSnapshot(errorCode,
                                               eventType,
                                               channel,
                                               source,
                                               flags,
                                               detail,
                                               lastErrorTimestampHours,
                                               canErrorLog.busOffCounter,
                                               wdgResetCounter);
}

static HAL_StatusTypeDef CAN_WriteDiagnosticLogEventSnapshot(uint8_t errorCode,
                                                             uint8_t eventType,
                                                             uint8_t channel,
                                                             uint8_t source,
                                                             uint32_t flags,
                                                             uint32_t detail,
                                                             uint16_t timestampHours,
                                                             uint16_t canBusOffCounter,
                                                             uint16_t watchdogCounter)
{
    HAL_StatusTypeDef status;

    status = DiagnosticLog_RecordEvent(errorCode,
                                       eventType,
                                       channel,
                                       source,
                                       flags,
                                       detail,
                                       timestampHours,
                                       canBusOffCounter,
                                       watchdogCounter);
    if (status == HAL_OK)
    {
        CAN_CheckLastDiagnosticLogRecord(errorCode);
    }

    return status;
}

static void CAN_CheckLastDiagnosticLogRecord(uint8_t sourceErrorCode)
{
    HAL_StatusTypeDef status;

    if (DiagnosticLog_CheckLastRecordCrc())
    {
        return;
    }

    CAN_RecordErrorCode(DIAG_LOG_ERROR_FLASH_CRC);

    if (sourceErrorCode == DIAG_LOG_ERROR_FLASH_CRC)
    {
        return;
    }

    status = DiagnosticLog_RecordEvent(DIAG_LOG_ERROR_FLASH_CRC,
                                       DIAG_LOG_EVENT_ERROR,
                                       DIAG_LOG_CHANNEL_GLOBAL,
                                       DIAG_LOG_SOURCE_FLASH,
                                       0u,
                                       0x43524332UL,
                                       lastErrorTimestampHours,
                                       canErrorLog.busOffCounter,
                                       wdgResetCounter);
    if (status == HAL_OK)
    {
        (void)DiagnosticLog_CheckLastRecordCrc();
    }
}

static void CAN_QueueDiagnosticLogEvent(uint8_t errorCode,
                                        uint8_t eventType,
                                        uint8_t channel,
                                        uint8_t source,
                                        uint32_t flags,
                                        uint32_t detail)
{
    /*
     * Эта функция может вызываться из CAN IRQ. Поэтому здесь только копия
     * короткого снимка в RAM; физическая запись Flash выполняется позже из
     * CanService_Task(), где уже можно безопасно блокироваться на erase/program.
     */
    pendingDiagnosticLog.errorCode = errorCode;
    pendingDiagnosticLog.eventType = eventType;
    pendingDiagnosticLog.channel = channel;
    pendingDiagnosticLog.source = source;
    pendingDiagnosticLog.timestampHours = lastErrorTimestampHours;
    pendingDiagnosticLog.canBusOffCounter = canErrorLog.busOffCounter;
    pendingDiagnosticLog.wdgCounter = wdgResetCounter;
    pendingDiagnosticLog.flags = flags;
    pendingDiagnosticLog.detail = detail;
    pendingDiagnosticLog.pending = true;
}

static void CAN_LoadPersistentDiagnostics(void)
{
    DiagnosticLogCounters_t counters = {0};

    DiagnosticLog_Init();
    DiagnosticLog_GetCounters(&counters);

    canErrorLog.busOffCounter = counters.can_busoff_counter;
    wdgResetCounter = counters.wdg_reset_counter;
    lastErrorTimestampHours = counters.last_error_timestamp_hours;
    lastErrorCode = counters.last_error_code;
}

static uint32_t CAN_BuildErrorLogFlags(uint32_t halError)
{
    uint32_t flags = 0u;

    if ((halError & HAL_CAN_ERROR_EWG) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_WARNING;
    }

    if ((halError & HAL_CAN_ERROR_EPV) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_PASSIVE;
    }

    if ((halError & HAL_CAN_ERROR_BOF) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_BUS_OFF;
    }

    if ((halError & HAL_CAN_ERROR_STF) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_STUFF;
    }

    if ((halError & HAL_CAN_ERROR_FOR) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_FORM;
    }

    if ((halError & HAL_CAN_ERROR_ACK) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_ACK;
    }

    if ((halError & HAL_CAN_ERROR_BR) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_BIT_RECESSIVE;
    }

    if ((halError & HAL_CAN_ERROR_BD) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_BIT_DOMINANT;
    }

    if ((halError & HAL_CAN_ERROR_CRC) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_CRC;
    }

    if ((halError & HAL_CAN_ERROR_RX_FOV0) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_RX_FIFO0_OVERRUN;
    }

    if ((halError & HAL_CAN_ERROR_RX_FOV1) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_RX_FIFO1_OVERRUN;
    }

    if ((halError & (HAL_CAN_ERROR_TX_ALST0 |
                     HAL_CAN_ERROR_TX_ALST1 |
                     HAL_CAN_ERROR_TX_ALST2)) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_TX_ARB_LOST;
    }

    if ((halError & (HAL_CAN_ERROR_TX_TERR0 |
                     HAL_CAN_ERROR_TX_TERR1 |
                     HAL_CAN_ERROR_TX_TERR2)) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_TX_ERROR;
    }

    if ((halError & HAL_CAN_ERROR_TIMEOUT) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_TIMEOUT;
    }

    if ((halError & (HAL_CAN_ERROR_NOT_INITIALIZED |
                     HAL_CAN_ERROR_NOT_READY |
                     HAL_CAN_ERROR_NOT_STARTED)) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_HAL_STATE;
    }

    if ((halError & HAL_CAN_ERROR_PARAM) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_PARAM;
    }

    if ((halError & HAL_CAN_ERROR_INTERNAL) != 0u)
    {
        flags |= CAN_ERROR_LOG_FLAG_INTERNAL;
    }

    return flags;
}

static void CAN_RecordErrorLog(uint32_t halError, uint32_t flags)
{
    if ((halError == HAL_CAN_ERROR_NONE) && (flags == 0u))
    {
        return;
    }

    canErrorLog.currentFlags = flags;
    canErrorLog.latchedFlags |= flags;
    canErrorLog.lastHalError = halError;
    canErrorLog.lastTimestampHours = (uint16_t)(HAL_GetTick() / 3600000u);

    if (canErrorLog.eventCounter < 0xFFFFFFFFu)
    {
        ++canErrorLog.eventCounter;
    }

    /*
     * BusOff-счетчик из ТЗ должен расти только при реальном Bus Off.
     * Раньше сюда насильно подставлялся BUS_OFF, из-за чего любая CAN-ошибка
     * превращалась в фальшивую запись журнала. Это ломало смысл диагностики.
     */
    if ((flags & CAN_ERROR_LOG_FLAG_BUS_OFF) != 0u)
    {
        if (canErrorLog.busOffCounter < 0xFFFFu)
        {
            ++canErrorLog.busOffCounter;
        }

        CAN_RecordErrorCode(DIAG_LOG_ERROR_CAN_BUS_OFF);
        CAN_QueueDiagnosticLogEvent(DIAG_LOG_ERROR_CAN_BUS_OFF,
                                    DIAG_LOG_EVENT_ERROR,
                                    DIAG_LOG_CHANNEL_GLOBAL,
                                    DIAG_LOG_SOURCE_CAN,
                                    flags,
                                    halError);
    }
}

static void CAN_QueueRxFrameFromIsr(const CAN_RxHeaderTypeDef *header, const uint8_t *frameData)
{
    uint8_t head;
    uint8_t nextHead;

    if ((header == NULL) || (frameData == NULL))
    {
        return;
    }

    head = canContext.rxHead;
    nextHead = (uint8_t)((head + 1u) % CAN_RX_QUEUE_SIZE);

    if (nextHead == canContext.rxTail)
    {
        CAN_RecordErrorLog(HAL_CAN_ERROR_NONE, CAN_ERROR_LOG_FLAG_RX_QUEUE_OVERFLOW);
        canContext.errorCode = 0x80000001u;
        canContext.errorDetected = true;
        canContext.dataReady = false;
        canContext.rxHead = 0u;
        canContext.rxTail = 0u;
        memset(canContext.rxQueue, 0, sizeof(canContext.rxQueue));
        return;
    }

    canContext.rxQueue[head].header = *header;
    memcpy(canContext.rxQueue[head].data, frameData, sizeof(canContext.rxQueue[head].data));
    canContext.rxHead = nextHead;
    canContext.dataReady = true;
}

static bool CAN_IsSupportedMsgId(uint16_t msgId)
{
    switch (msgId)
    {
        case CAN_MSG_INA260_VOLTAGE:
        case CAN_MSG_INA260_CURRENT:
        case CAN_MSG_BAT_SOC:
        case CAN_MSG_BQ_STATUS:
        case CAN_MSG_ERROR_FLAGS:
        case CAN_MSG_UART_VERSION:
        case CAN_MSG_BAT_CAPACITY_MAH:
        case CAN_MSG_BAT_NOMINAL_CAP:
        case CAN_MSG_BQ_COULOMB_COUNT_MAH:
        case CAN_MSG_CAN_BUSOFF_COUNTER:
        case CAN_MSG_WDG_RESET_COUNTER:
        case CAN_MSG_LAST_ERROR_TIMESTAMP:
        case CAN_MSG_LAST_ERROR_CODE:
        case CAN_MSG_DIAG_LOG_INFO:
        case CAN_MSG_DIAG_LOG_READ:
            return true;

        default:
            return false;
    }
}

static uint8_t CAN_ResponsePriorityForMsgId(uint16_t msgId)
{
    switch (msgId)
    {
        case CAN_MSG_CAN_BUSOFF_COUNTER:
        case CAN_MSG_WDG_RESET_COUNTER:
        case CAN_MSG_LAST_ERROR_TIMESTAMP:
        case CAN_MSG_LAST_ERROR_CODE:
        case CAN_MSG_DIAG_LOG_INFO:
        case CAN_MSG_DIAG_LOG_READ:
            return CAN_PRIORITY_DIAGNOSTIC;

        default:
            return CAN_PRIORITY_DEFAULT;
    }
}

static void CAN_PackWordLE(uint8_t *dst, uint16_t value)
{
    if (dst == NULL)
    {
        return;
    }

    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8);
}

static uint16_t CAN_GetBalanceDeltaMv(const ModbusSlaveData_t *data)
{
    uint16_t minCell = 0xFFFFu;
    uint16_t maxCell = 0u;

    if (data == NULL)
    {
        return 0u;
    }

    for (size_t index = 0u; index < 10u; ++index)
    {
        uint16_t cellMv = data->cell_voltage_mv[index];

        if (cellMv < minCell)
        {
            minCell = cellMv;
        }

        if (cellMv > maxCell)
        {
            maxCell = cellMv;
        }
    }

    return (uint16_t)(maxCell - minCell);
}

static uint16_t CAN_ComposeErrorFlags(uint8_t channelIndex)
{
    const ModbusSlaveData_t *data;
    uint16_t flags = 0u;
    uint16_t balanceDeltaMv;

    if (channelIndex >= UART_CHANNEL_COUNT)
    {
        return CAN_ERROR_FLAG_COMM_ERROR;
    }

    data = &modbusSlaveData[channelIndex];
    if (!data->valid)
    {
        return CAN_ERROR_FLAG_COMM_ERROR;
    }

    flags = (uint16_t)(data->error_flags & (CAN_ERROR_FLAG_BQ_ERROR | CAN_ERROR_FLAG_INA_ERROR));
    balanceDeltaMv = CAN_GetBalanceDeltaMv(data);

    if (data->soc_percent == 100u)
    {
        if (balanceDeltaMv >= CAN_BALANCE_WARNING_THRESHOLD_MV)
        {
            flags |= CAN_ERROR_FLAG_BALANCE_WARNING;
        }

        if (balanceDeltaMv > CAN_BALANCE_CRITICAL_THRESHOLD_MV)
        {
            flags |= CAN_ERROR_FLAG_BALANCE_CRITICAL;
        }
    }

    return flags;
}

static uint16_t CAN_GetUnavailableWord(void)
{
    return 0xFFFFu;
}

static uint16_t CAN_GetRegisterResponseWord(uint8_t channelIndex, uint16_t msgId)
{
    const ModbusSlaveData_t *data;

    if (channelIndex >= UART_CHANNEL_COUNT)
    {
        return (msgId == CAN_MSG_ERROR_FLAGS) ? CAN_ERROR_FLAG_COMM_ERROR : CAN_GetUnavailableWord();
    }

    if (msgId == CAN_MSG_ERROR_FLAGS)
    {
        return CAN_ComposeErrorFlags(channelIndex);
    }

    data = &modbusSlaveData[channelIndex];
    if (!data->valid)
    {
        return CAN_GetUnavailableWord();
    }

    switch (msgId)
    {
        case CAN_MSG_INA260_VOLTAGE:
            return data->voltage_mv;

        case CAN_MSG_INA260_CURRENT:
            return (uint16_t)data->current_ma;

        case CAN_MSG_BAT_SOC:
            return data->soc_percent;

        case CAN_MSG_BQ_STATUS:
            return data->bq_status_state_sys;

        case CAN_MSG_UART_VERSION:
            return data->firmware_version;

        case CAN_MSG_BAT_CAPACITY_MAH:
            return data->capacity_mah;

        case CAN_MSG_BAT_NOMINAL_CAP:
            return data->nominal_capacity_mah;

        case CAN_MSG_BQ_COULOMB_COUNT_MAH:
            return data->bq_coulomb_count_mah;

        default:
            return CAN_GetUnavailableWord();
    }
}

static uint16_t CAN_GetDiagnosticWord(uint16_t msgId)
{
    switch (msgId)
    {
        case CAN_MSG_CAN_BUSOFF_COUNTER:
            return canErrorLog.busOffCounter;

        case CAN_MSG_WDG_RESET_COUNTER:
            return wdgResetCounter;

        case CAN_MSG_LAST_ERROR_TIMESTAMP:
            return lastErrorTimestampHours;

        default:
            return 0u;
    }
}

static uint8_t CAN_GetResponseDlc(uint16_t msgId)
{
    return (msgId == CAN_MSG_LAST_ERROR_CODE) ? 4u : 8u;
}

static void CAN_BuildDiagnosticLogInfoPayload(uint8_t payload[8])
{
    DiagnosticLogExportInfo_t info = {0};

    if (payload == NULL)
    {
        return;
    }

    memset(payload, 0, 8u);
    DiagnosticLog_GetExportInfo(&info);

    /*
     * MSG_ID=34: короткая сводка для ПК перед выгрузкой журнала.
     * 0..1 capacity, 2 record_size, 3 chunks, 4..5 valid_count, 6..7 last_index.
     */
    CAN_PackWordLE(&payload[0], info.record_capacity);
    payload[2] = info.record_size_bytes;
    payload[3] = info.chunks_per_record;
    CAN_PackWordLE(&payload[4], info.valid_record_count);
    CAN_PackWordLE(&payload[6], info.last_record_index);
}

static void CAN_BuildDiagnosticLogReadPayload(const uint8_t *requestData,
                                              uint8_t requestDlc,
                                              uint8_t payload[8])
{
    uint16_t recordIndex;
    uint8_t chunkIndex;

    if (payload == NULL)
    {
        return;
    }

    memset(payload, 0xFF, 8u);

    if ((requestData == NULL) || (requestDlc < 3u))
    {
        return;
    }

    recordIndex = (uint16_t)requestData[0] | ((uint16_t)requestData[1] << 8);
    chunkIndex = requestData[2];

    /*
     * MSG_ID=35: request[0..1] = физический индекс записи, request[2] = chunk 0..3.
     * Ответом идут ровно 8 сырых байт физического слота. Если запрос битый,
     * оставляем 0xFF; пустой слот сам выглядит как 0xFF.
     */
    (void)DiagnosticLog_ReadRecordChunk(recordIndex, chunkIndex, payload);
}

static void CAN_BuildResponsePayload(uint16_t msgId, uint8_t payload[8])
{
    if (payload == NULL)
    {
        return;
    }

    memset(payload, 0, 8u);

    if (msgId == CAN_MSG_LAST_ERROR_CODE)
    {
        for (size_t index = 0u; index < UART_CHANNEL_COUNT; ++index)
        {
            payload[index] = lastErrorCode;
        }

        return;
    }

    if ((msgId == CAN_MSG_CAN_BUSOFF_COUNTER) ||
        (msgId == CAN_MSG_WDG_RESET_COUNTER) ||
        (msgId == CAN_MSG_LAST_ERROR_TIMESTAMP))
    {
        uint16_t diagnosticValue = CAN_GetDiagnosticWord(msgId);

        for (size_t index = 0u; index < UART_CHANNEL_COUNT; ++index)
        {
            CAN_PackWordLE(&payload[index * 2u], diagnosticValue);
        }

        return;
    }

    for (size_t index = 0u; index < UART_CHANNEL_COUNT; ++index)
    {
        uint16_t value = CAN_GetRegisterResponseWord((uint8_t)index, msgId);
        CAN_PackWordLE(&payload[index * 2u], value);
    }
}

static void CAN_SendFrame(CAN_HandleTypeDef *hcan,
                          uint32_t extId,
                          const uint8_t *data,
                          uint8_t dlc)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint32_t txMailbox = 0u;
    uint8_t txData[8] = {0};

    if (hcan == NULL)
    {
        return;
    }

    if (dlc > 8u)
    {
        dlc = 8u;
    }

    if ((data != NULL) && (dlc > 0u))
    {
        memcpy(txData, data, dlc);
    }

    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_EXT;
    txHeader.DLC = dlc;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.ExtId = extId;

    if ((canContext.mutex != NULL) && (osMutexWait(canContext.mutex, osWaitForever) == osOK))
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0u)
        {
            if (HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox) != HAL_OK)
            {
                taskENTER_CRITICAL();
                CAN_RecordErrorLog(hcan->ErrorCode, CAN_BuildErrorLogFlags(hcan->ErrorCode));
                taskEXIT_CRITICAL();
                lastProcessedCanError = hcan->ErrorCode;
            }
        }
        else
        {
            taskENTER_CRITICAL();
            CAN_RecordErrorLog(hcan->ErrorCode, CAN_ERROR_LOG_FLAG_TX_MAILBOX_FULL);
            taskEXIT_CRITICAL();
            lastProcessedCanError = hcan->ErrorCode;
        }

        osMutexRelease(canContext.mutex);
    }
}

void MX_CAN1_Init(void)
{
    CAN_LoadPersistentDiagnostics();
    CAN_RecordStartupResetFlags();

    __HAL_RCC_CAN1_FORCE_RESET();
    HAL_Delay(10);
    __HAL_RCC_CAN1_RELEASE_RESET();
    HAL_Delay(10);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 5;
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

    canContext.mutex = osMutexCreate(osMutex(CANBufferMutex));
    if (canContext.mutex == NULL)
    {
        Error_Handler();
    }

    if (CAN_ConfigHardwareFilters() != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
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

uint32_t CAN_BuildExtId(uint8_t src, uint8_t dst, uint16_t msgId, uint8_t priority)
{
    return (((uint32_t)src & 0x7Fu) |
            (((uint32_t)dst & 0x7Fu) << 7) |
            (((uint32_t)msgId & 0x03FFu) << 14) |
            (((uint32_t)priority & 0x03u) << 27));
}

bool CAN_ParseExtId(uint32_t canId, uint8_t *src, uint8_t *dst, uint16_t *msgId, uint8_t *priority)
{
    if (src != NULL)
    {
        *src = (uint8_t)(canId & 0x7Fu);
    }

    if (dst != NULL)
    {
        *dst = (uint8_t)((canId >> 7) & 0x7Fu);
    }

    if (msgId != NULL)
    {
        *msgId = (uint16_t)((canId >> 14) & 0x03FFu);
    }

    if (priority != NULL)
    {
        *priority = (uint8_t)((canId >> 27) & 0x03u);
    }

    return true;
}

bool CAN_HandleRegisterRequest(CAN_HandleTypeDef *hcan,
                               uint32_t extId,
                               const uint8_t *requestData,
                               uint8_t requestDlc)
{
    uint8_t src = 0u;
    uint8_t dst = 0u;
    uint8_t requestPriority = 0u;
    uint16_t msgId = 0u;
    uint8_t responsePayload[8] = {0};
    uint32_t responseId;

    CAN_ParseExtId(extId, &src, &dst, &msgId, &requestPriority);

    if ((src != CAN_NODE_KOU) || (dst != CAN_NODE_IBP_4K))
    {
        return false;
    }

    if (!CAN_IsSupportedMsgId(msgId))
    {
        return false;
    }

    if (msgId == CAN_MSG_DIAG_LOG_INFO)
    {
        CAN_BuildDiagnosticLogInfoPayload(responsePayload);
    }
    else if (msgId == CAN_MSG_DIAG_LOG_READ)
    {
        CAN_BuildDiagnosticLogReadPayload(requestData, requestDlc, responsePayload);
    }
    else
    {
        CAN_BuildResponsePayload(msgId, responsePayload);
    }

    responseId = CAN_BuildExtId(CAN_NODE_IBP_4K,
                                CAN_NODE_KOU,
                                msgId,
                                CAN_ResponsePriorityForMsgId(msgId));

    CAN_SendExtendedFrame(hcan, responseId, responsePayload, CAN_GetResponseDlc(msgId));

    return true;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t frameData[8] = {0};

    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frameData) != HAL_OK)
    {
        return;
    }

    {
        UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
        CAN_QueueRxFrameFromIsr(&header, frameData);
        taskEXIT_CRITICAL_FROM_ISR(irqState);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t frameData[8] = {0};

    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, frameData) != HAL_OK)
    {
        return;
    }

    {
        UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
        CAN_QueueRxFrameFromIsr(&header, frameData);
        taskEXIT_CRITICAL_FROM_ISR(irqState);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t errorFlags;

    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    errorFlags = CAN_BuildErrorLogFlags(hcan->ErrorCode);

    {
        UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
        CAN_RecordErrorLog(hcan->ErrorCode, errorFlags);
        canContext.errorCode = hcan->ErrorCode;
        canContext.errorDetected = true;
        canContext.dataReady = false;
        canContext.rxHead = 0u;
        canContext.rxTail = 0u;
        memset(canContext.rxQueue, 0, sizeof(canContext.rxQueue));
        taskEXIT_CRITICAL_FROM_ISR(irqState);
    }
}

bool CAN_DequeueReceivedFrame(CAN_RxHeaderTypeDef *header, uint8_t *data, uint8_t *dlc)
{
    bool hasFrame = false;

    if ((header == NULL) || (data == NULL) || (dlc == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (canContext.rxHead != canContext.rxTail)
    {
        const CanRxFrame_t *frame = &canContext.rxQueue[canContext.rxTail];

        *header = frame->header;
        memcpy(data, frame->data, sizeof(frame->data));
        canContext.rxTail = (uint8_t)((canContext.rxTail + 1u) % CAN_RX_QUEUE_SIZE);
        canContext.dataReady = (canContext.rxHead != canContext.rxTail);
        *dlc = (header->DLC <= 8u) ? header->DLC : 8u;
        hasFrame = true;
    }

    taskEXIT_CRITICAL();

    return hasFrame;
}

void CAN_SendExtendedFrame(CAN_HandleTypeDef *hcan,
                           uint32_t extId,
                           const uint8_t *data,
                           uint8_t dlc)
{
    CAN_SendFrame(hcan, extId, data, dlc);
}

void CAN_ReportFlashWriteError(void)
{
    CAN_RecordErrorCode(DIAG_LOG_ERROR_FLASH_WRITE);
    (void)CAN_WriteDiagnosticLogEvent(DIAG_LOG_ERROR_FLASH_WRITE,
                                      DIAG_LOG_EVENT_ERROR,
                                      DIAG_LOG_CHANNEL_GLOBAL,
                                      DIAG_LOG_SOURCE_FLASH,
                                      0u,
                                      HAL_FLASH_GetError());
}

void CAN_ProcessPendingDiagnosticLog(void)
{
    bool hasPendingEvent = false;
    uint8_t errorCode = 0u;
    uint8_t eventType = 0u;
    uint8_t channel = 0u;
    uint8_t source = 0u;
    uint16_t timestampHours = 0u;
    uint16_t busOffCounter = 0u;
    uint16_t watchdogCounter = 0u;
    uint32_t flags = 0u;
    uint32_t detail = 0u;
    HAL_StatusTypeDef status;

    taskENTER_CRITICAL();

    if (pendingDiagnosticLog.pending)
    {
        errorCode = pendingDiagnosticLog.errorCode;
        eventType = pendingDiagnosticLog.eventType;
        channel = pendingDiagnosticLog.channel;
        source = pendingDiagnosticLog.source;
        timestampHours = pendingDiagnosticLog.timestampHours;
        busOffCounter = pendingDiagnosticLog.canBusOffCounter;
        watchdogCounter = pendingDiagnosticLog.wdgCounter;
        flags = pendingDiagnosticLog.flags;
        detail = pendingDiagnosticLog.detail;
        pendingDiagnosticLog.pending = false;
        hasPendingEvent = true;
    }

    taskEXIT_CRITICAL();

    if (!hasPendingEvent)
    {
        return;
    }

    status = CAN_WriteDiagnosticLogEventSnapshot(errorCode,
                                                 eventType,
                                                 channel,
                                                 source,
                                                 flags,
                                                 detail,
                                                 timestampHours,
                                                 busOffCounter,
                                                 watchdogCounter);
    if (status != HAL_OK)
    {
        /*
         * Не пытаемся логировать ошибку записи журнала еще одной записью:
         * так легко получить рекурсию "ошибка записи ошибки записи".
         */
        CAN_RecordErrorCode(DIAG_LOG_ERROR_FLASH_WRITE);
    }
}
