#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Основная задача прикладной обработки принятых CAN-кадров. */
void CanService_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* CAN_SERVICE_H */
