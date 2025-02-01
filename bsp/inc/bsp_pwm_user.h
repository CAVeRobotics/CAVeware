#ifndef BSP_PWM_USER_H
#define BSP_PWM_USER_H

#include "bsp_types.h"

typedef enum
{
    BSP_PWM_USER_TIMER_STEERING_SERVOS,
    BSP_PWM_USER_TIMER_MAX
} BspPwmUser_Timer_t;

typedef struct
{
    BspTypes_TimerHandle_t *timer_handle;
    BspTypes_TimerChannel_t max_channel;
} BspPwmUser_TimerConfig_t;

#endif /* BSP_PWM_USER_H */