#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

#include "cavebot.h"

typedef uint32_t Scheduler_Tick_t;

Cavebot_Error_t Scheduler_Initialize(void);
Cavebot_Error_t Scheduler_AddTask(void (*task)(void), const Scheduler_Tick_t interval);
Cavebot_Error_t Scheduler_Start(void);
Cavebot_Error_t Scheduler_Stop(void);
void Scheduler_Run(void);

#endif /* SCHEDULER_H */