#ifndef CAVEBOT_CAVETALK_H
#define CAVEBOT_CAVETALK_H

#include <stddef.h>

#include "bsp.h"
#include "cave_talk.h"

CaveTalk_Error_t CavebotCaveTalk_Start(void);
void CavebotCaveTalk_Task(void);

#endif /* CAVEBOT_CAVETALK_H */