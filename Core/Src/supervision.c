#include "supervision.h"

#include "battery_data.h"
#include "board_io.h"
#include "cmsis_os.h"
#include "main.h"

/* Основная задача обслуживания watchdog, IPS-входов и таймаутов связи. */
void Supervision_Task(void const *argument)
{
    uint32_t lastIpsPollTick;

    (void)argument;

    Board_UpdateIpsInputs();
    lastIpsPollTick = HAL_GetTick();

    for (;;)
    {
        uint32_t currentTick;

        Board_SetWatchdogPin(1u);
        osDelay(100);
        Board_SetWatchdogPin(0u);
        osDelay(100);

        CheckConnectionTimeout();
        ++wdi_tick;

        currentTick = HAL_GetTick();
        if ((currentTick - lastIpsPollTick) >= 1000u)
        {
            Board_UpdateIpsInputs();
            lastIpsPollTick = currentTick;
        }
    }
}
