#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Включает прикладную работу UART-каналов с батарейными блоками. */
#define UARTS_ENABLE 1

/* Включает прикладную обработку CAN-кадров. */
#define CAN_ENABLE   1

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
