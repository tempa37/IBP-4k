#include "can_service.h"

#include "app_config.h"
#include "app_diagnostics.h"
#include "board_io.h"
#include "can.h"
#include "cmsis_os.h"
#include "firmware_update.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/* Сбрасывает прикладный флаг ошибки CAN и сохраняет его для отладки. */
static void CanService_HandleErrorState(void)
{
    if (!canContext.errorDetected)
    {
        return;
    }

    taskENTER_CRITICAL();
    lastProcessedCanError = canContext.errorCode;
    lastProcessedCanId = 0u;
    lastProcessedCanDlc = 0u;
    canContext.errorDetected = false;
    canContext.errorCode = 0u;
    taskEXIT_CRITICAL();
}

/* Диспетчеризует один принятый extended CAN-кадр между update и register слоями. */
static void CanService_HandleExtendedFrame(const CAN_RxHeaderTypeDef *header,
                                           const uint8_t *data,
                                           uint8_t dataLength)
{
    if ((header == NULL) || (data == NULL))
    {
        return;
    }

    if (!FW_HandleExtendedUpdateCommand(header->ExtId, data, dataLength))
    {
        (void)CAN_HandleRegisterRequest(&hcan1, header->ExtId);
    }
}

/* Основная задача прикладной обработки входящих CAN-кадров. */
void CanService_Task(void const *argument)
{
    CAN_RxHeaderTypeDef localHeader = {0};
    uint8_t dataLength = 0u;

    (void)argument;
    Board_InitRuntimeOutputs();

    for (;;)
    {
#if CAN_ENABLE == 1
        CanService_HandleErrorState();
        CAN_ProcessPendingDiagnosticLog();

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
                CanService_HandleExtendedFrame(&localHeader, CanlocalData, dataLength);
            }
        }
#endif

        osDelay(1);
    }
}
