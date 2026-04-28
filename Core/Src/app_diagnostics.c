#include "app_diagnostics.h"

/* Номер UART, с которого последним был обработан пользовательский кадр. */
volatile uint8_t lastProcessedUartNumber = 0u;

/* Последний обработанный расширенный CAN ID для отладки обмена. */
volatile uint32_t lastProcessedCanId = 0u;

/* Последняя длина обработанного CAN-кадра для отладки обмена. */
volatile uint8_t lastProcessedCanDlc = 0u;

/* Последняя ошибка UART, поднятая прикладным обработчиком. */
volatile uint32_t lastProcessedUartError = 0u;

/* Последняя ошибка CAN, поднятая прикладным обработчиком. */
volatile uint32_t lastProcessedCanError = 0u;

/* Последнее рассчитанное значение CRC32 принятого master-образа. */
volatile uint32_t crc32 = 0u;

/* Последнее эталонное значение CRC32, прочитанное из принятого образа. */
volatile uint32_t iar_crc32 = 0u;
