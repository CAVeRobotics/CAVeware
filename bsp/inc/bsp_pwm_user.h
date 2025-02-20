#ifndef BSP_PWM_USER_H
#define BSP_PWM_USER_H

#include "bsp.h"

typedef enum
{
    BSP_PWM_USER_TIMER_MOTORS,
    BSP_PWM_USER_TIMER_STEERING_SERVOS,
    BSP_PWM_USER_TIMER_CAMERA_SERVOS,
    BSP_PWM_USER_TIMER_MAX
} BspPwmUser_Timer_t;

typedef struct
{
    Bsp_TimerHandle_t *timer_handle;
    Bsp_TimerChannel_t max_channel;
} BspPwmUser_TimerConfig_t;

extern const BspPwmUser_TimerConfig_t BspPwmUser_TimerConfigTable[BSP_PWM_USER_TIMER_MAX];

#endif /* BSP_PWM_USER_H */