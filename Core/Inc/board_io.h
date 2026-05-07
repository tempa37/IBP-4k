#ifndef BOARD_IO_H
#define BOARD_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile uint8_t ips1_in;
extern volatile uint8_t ips2_in;
extern volatile uint8_t ips3_in;
extern volatile uint8_t ips4_in;
extern uint32_t wdi_tick;

/* Инициализирует прикладные выходы платы после настройки GPIO. */
void Board_InitRuntimeOutputs(void);

/* Обновляет RAM-состояние дискретных входов IPS. */
void Board_UpdateIpsInputs(void);

/* Управляет выходом внешнего watchdog-индикатора. */
void Board_SetWatchdogPin(uint8_t enabled);

/* Устанавливает LED-индикацию выбранного батарейного канала. */
void BAT_SetIndicator(uint8_t battery);

/* Обновляет LED-индикацию всех батарейных каналов. */
void BAT_UpdateAllIndicators(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_IO_H */
