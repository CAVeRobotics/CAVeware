#include "rover_camera_config.h"

#include "bsp.h"
#include "bsp_pwm_user.h"
#include "bsp_servo.h"

#include "rover.h"

BspServo_Handle_t RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_MAX] = {
    [ROVER_CAMERA_CONFIG_SERVO_PAN] = {
        .timer              = BSP_PWM_USER_TIMER_CAMERA_PAN,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125 / 2.0,
        .minimum_angle      = 0,
        .maximum_angle      = 180 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_CAMERA_CONFIG_SERVO_TILT] = {
        .timer              = BSP_PWM_USER_TIMER_CAMERA_TILT,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 180 * ROVER_DEGREES_TO_RADIANS,
    },
};