#ifndef SUPERVISION_H
#define SUPERVISION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Основная задача обслуживания watchdog, IPS-входов и таймаутов связи. */
void Supervision_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* SUPERVISION_H */
