#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile uint8_t lastProcessedUartNumber;
extern volatile uint32_t lastProcessedCanId;
extern volatile uint8_t lastProcessedCanDlc;
extern volatile uint32_t lastProcessedUartError;
extern volatile uint32_t lastProcessedCanError;
extern volatile uint32_t crc32;
extern volatile uint32_t iar_crc32;

#ifdef __cplusplus
}
#endif

#endif /* APP_DIAGNOSTICS_H */
