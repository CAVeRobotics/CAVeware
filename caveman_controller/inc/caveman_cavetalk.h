#ifndef CAVEMAN_CAVETALK_H
#define CAVEMAN_CAVETALK_H

#include <stddef.h>

#include "bsp.h"
#include "cave_talk.h"

CaveTalk_Error_t CavemanCaveTalk_Start(void);
void CavemanCaveTalk_Task(void);

#endif /* CAVEMAN_CAVETALK_H */