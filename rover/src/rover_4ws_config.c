#include "rover_4ws_config.h"

#include "bsp.h"
#include "bsp_pwm_user.h"
#include "bsp_servo.h"

#include "rover.h"

const Rover_Meter_t kRover4wsConfig_Tread       = 0.493800;
const Rover_Meter_t kRover4wsConfig_Wheelbase   = 0.466028;
const Rover_Meter_t kRover4wsConfig_WheelRadius = 0.080000;

const Rover_Meter_t kRover4wsConfig_HalfTread         = kRover4wsConfig_Tread / 2;
const Rover_Meter_t kRover4wsConfig_HalfWheelbase     = kRover4wsConfig_Wheelbase / 2;
const Rover_Meter_t kRover4wsConfig_DoubleWheelRadius = kRover4wsConfig_WheelRadius * 2;

const BspServo_Handle_t Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_MAX] = {
    [ROVER_4WS_CONFIG_SERVO_0] = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_4WS_CONFIG_SERVO_1] = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_4WS_CONFIG_SERVO_2] = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_3,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_4WS_CONFIG_SERVO_3] = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_4,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120 * ROVER_DEGREES_TO_RADIANS,
    },
};