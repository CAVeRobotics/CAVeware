#include "bsp_pwm_user.h"

#include "tim.h"

#include "bsp.h"

const BspPwmUser_TimerConfig_t BspPwmUser_TimerConfigTable[BSP_PWM_USER_TIMER_MAX] = {
    [BSP_PWM_USER_TIMER_MOTORS] = {
        .timer_handle = &htim1, .max_channel = BSP_TIMER_CHANNEL_4
    },
    [BSP_PWM_USER_TIMER_STEERING_SERVOS] = {
        .timer_handle = &htim4, .max_channel = BSP_TIMER_CHANNEL_4
    },
    [BSP_PWM_USER_TIMER_CAMERA_SERVOS] = {
        .timer_handle = &htim2, .max_channel = BSP_TIMER_CHANNEL_3
    },
};