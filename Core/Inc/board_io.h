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

void Board_InitRuntimeOutputs(void);
void Board_UpdateIpsInputs(void);
void Board_SetWatchdogPin(uint8_t enabled);
void BAT_SetIndicator(uint8_t battery);
void BAT_UpdateAllIndicators(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_IO_H */
