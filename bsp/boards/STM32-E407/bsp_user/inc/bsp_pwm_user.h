#ifndef BSP_PWM_USER_H
#define BSP_PWM_USER_H

#include "bsp.h"

typedef enum
{
    BSP_PWM_USER_TIMER_MOTORS,
    BSP_PWM_USER_TIMER_MAX
} BspPwmUser_Timer_t;

extern const Bsp_PwmConfig_t BspPwmUser_TimerConfigTable[1U];

#endif /* BSP_PWM_USER_H */