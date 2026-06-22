#ifndef BSP_TIMER_USER_H
#define BSP_TIMER_USER_H

#include "bsp.h"

typedef enum
{
    BSP_TIMER_USER_TIMER_0,
    BSP_TIMER_USER_TIMER_MAX
} BspTimerUser_Timer_t;

extern Bsp_Timer_t BspTimerUser_HandleTable[BSP_TIMER_USER_TIMER_MAX];

#endif /* BSP_TIMER_USER_H */