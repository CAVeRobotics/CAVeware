#include "bsp_pwm_user.h"

#include "bsp.h"

extern Bsp_TimerHandle_t htim4;

BspPwmUser_TimerConfig_t BspPwmUser_TimerConfigTable[BSP_PWM_USER_TIMER_MAX] = {
    [BSP_PWM_USER_TIMER_STEERING_SERVOS] = {
        .timer_handle = &htim4, .max_channel = BSP_TIMER_CHANNEL_4
    },
};