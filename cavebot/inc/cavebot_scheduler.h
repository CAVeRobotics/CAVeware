#ifndef CAVEBOT_SCHEDULER_H
#define CAVEBOT_SCHEDULER_H

#include <stdint.h>

#include "cavebot.h"

typedef uint32_t CavebotScheduler_Tick_t;

Cavebot_Error_t CavebotScheduler_Initialize(void);
Cavebot_Error_t CavebotScheduler_AddTask(void (*task)(void), const CavebotScheduler_Tick_t interval);
Cavebot_Error_t CavebotScheduler_Start(void);
Cavebot_Error_t CavebotScheduler_Stop(void);
void CavebotScheduler_Run(void);

#endif /* CAVEBOT_SCHEDULER_H */