#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t Scheduler_Tick_t;

bool Scheduler_Initialize(void);
bool Scheduler_AddTask(void (*task)(void), const Scheduler_Tick_t interval);
bool Scheduler_Start(void);
bool Scheduler_Stop(void);
void Scheduler_Run(void);

#endif /* SCHEDULER_H */