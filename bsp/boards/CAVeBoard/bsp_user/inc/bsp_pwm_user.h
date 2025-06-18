#ifndef BSP_PWM_USER_H
#define BSP_PWM_USER_H

#include "bsp.h"

typedef enum
{
    BSP_PWM_USER_TIMER_0,
    BSP_PWM_USER_TIMER_1,
    BSP_PWM_USER_TIMER_2,
    BSP_PWM_USER_TIMER_3,
    BSP_PWM_USER_TIMER_4,
    BSP_PWM_USER_TIMER_5,
    BSP_PWM_USER_TIMER_MAX
} BspPwmUser_Timer_t;

extern const Bsp_PwmConfig_t BspPwmUser_TimerConfigTable[BSP_PWM_USER_TIMER_MAX];

#endif /* BSP_PWM_USER_H */