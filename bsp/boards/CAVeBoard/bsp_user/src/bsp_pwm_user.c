#include "bsp_pwm_user.h"

#include "tim.h"

#include "bsp.h"

const Bsp_PwmConfig_t BspPwmUser_TimerConfigTable[BSP_PWM_USER_TIMER_MAX] = {
    [BSP_PWM_USER_TIMER_MOTORS_FRONT] = {
        .timer_handle = &htim1, .max_channel = BSP_TIMER_CHANNEL_4
    },
    [BSP_PWM_USER_TIMER_MOTORS_REAR] = {
        .timer_handle = &htim8, .max_channel = BSP_TIMER_CHANNEL_4
    },
    [BSP_PWM_USER_TIMER_SERVOS_FRONT] = {
        .timer_handle = &htim9, .max_channel = BSP_TIMER_CHANNEL_2
    },
    [BSP_PWM_USER_TIMER_SERVOS_REAR] = {
        .timer_handle = &htim12, .max_channel = BSP_TIMER_CHANNEL_2
    },
    [BSP_PWM_USER_TIMER_CAMERA_PAN] = {
        .timer_handle = &htim10, .max_channel = BSP_TIMER_CHANNEL_1
    },
    [BSP_PWM_USER_TIMER_CAMERA_TILT] = {
        .timer_handle = &htim11, .max_channel = BSP_TIMER_CHANNEL_1
    },
};