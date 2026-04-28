#include "board_io.h"

#include "battery_data.h"
#include "main.h"
#include "uart.h"

volatile uint8_t ips1_in = 0u;
volatile uint8_t ips2_in = 0u;
volatile uint8_t ips3_in = 0u;
volatile uint8_t ips4_in = 0u;
uint32_t wdi_tick = 0u;

/* Настраивает прикладные выходы платы после инициализации GPIO. */
void Board_InitRuntimeOutputs(void)
{
    HAL_GPIO_WritePin(GPIOG,
                      EN_1_Pin | EN_2_Pin | EN_3_Pin | EN_4_Pin,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_ON2_GPIO_Port, RS485_ON2_Pin, GPIO_PIN_RESET);
}

/* Обновляет RAM-копию состояния дискретных входов IPS. */
void Board_UpdateIpsInputs(void)
{
    ips1_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_1_GPIO_Port, IPS_1_Pin) == GPIO_PIN_SET);
    ips2_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_2_GPIO_Port, IPS_2_Pin) == GPIO_PIN_SET);
    ips3_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_3_GPIO_Port, IPS_3_Pin) == GPIO_PIN_SET);
    ips4_in = (uint8_t)(HAL_GPIO_ReadPin(IPS_4_GPIO_Port, IPS_4_Pin) == GPIO_PIN_SET);
}

/* Выставляет уровень на входе внешнего watchdog-индикатора. */
void Board_SetWatchdogPin(uint8_t enabled)
{
    HAL_GPIO_WritePin(WDI_GPIO_Port,
                      WDI_Pin,
                      (enabled != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Управляет LED-индикатором выбранного батарейного канала. */
void BAT_SetIndicator(uint8_t battery)
{
    GPIO_PinState state = GPIO_PIN_SET;

    if (battery >= UART_CHANNEL_COUNT)
    {
        return;
    }

    if (modbusSlaveData[battery].valid)
    {
        state = GPIO_PIN_RESET;
    }

    switch (battery)
    {
        case UART_CHANNEL_8:
            HAL_GPIO_WritePin(BAT_1_GPIO_Port, BAT_1_Pin, state);
            break;

        case UART_CHANNEL_7:
            HAL_GPIO_WritePin(BAT_2_GPIO_Port, BAT_2_Pin, state);
            break;

        case UART_CHANNEL_4:
            HAL_GPIO_WritePin(BAT_3_GPIO_Port, BAT_3_Pin, state);
            break;

        case UART_CHANNEL_5:
            HAL_GPIO_WritePin(BAT_4_GPIO_Port, BAT_4_Pin, state);
            break;

        default:
            break;
    }
}

/* Обновляет LED-индикаторы всех батарейных каналов. */
void BAT_UpdateAllIndicators(void)
{
    for (uint8_t index = 0u; index < UART_CHANNEL_COUNT; ++index)
    {
        BAT_SetIndicator(index);
    }
}
