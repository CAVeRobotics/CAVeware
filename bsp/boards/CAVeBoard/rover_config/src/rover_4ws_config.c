#include "rover_4ws_config.h"

#include "bsp.h"
#include "bsp_encoder_user.h"
#include "bsp_motor.h"
#include "bsp_pwm_user.h"
#include "bsp_servo.h"

#include "rover.h"

const Rover_Meter_t kRover4wsConfig_Tread       = 0.493800;
const Rover_Meter_t kRover4wsConfig_Wheelbase   = 0.466028;
const Rover_Meter_t kRover4wsConfig_WheelRadius = 0.080000;

const Rover_Meter_t kRover4wsConfig_HalfTread         = kRover4wsConfig_Tread / 2;
const Rover_Meter_t kRover4wsConfig_HalfWheelbase     = kRover4wsConfig_Wheelbase / 2;
const Rover_Meter_t kRover4wsConfig_DoubleWheelRadius = kRover4wsConfig_WheelRadius * 2;

BspServo_Handle_t Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_MAX] = {
    [ROVER_4WS_CONFIG_SERVO_0] = {
        .timer              = BSP_PWM_USER_TIMER_SERVOS_FRONT,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.115,
        .minimum_angle      = 10 * ROVER_DEGREES_TO_RADIANS,
        .maximum_angle      = 170 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_4WS_CONFIG_SERVO_1] = {
        .timer              = BSP_PWM_USER_TIMER_SERVOS_FRONT,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.1125,
        .minimum_angle      = 10 * ROVER_DEGREES_TO_RADIANS,
        .maximum_angle      = 170 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_4WS_CONFIG_SERVO_2] = {
        .timer              = BSP_PWM_USER_TIMER_SERVOS_REAR,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.03,
        .maximum_duty_cycle = 0.1075,
        .minimum_angle      = 10 * ROVER_DEGREES_TO_RADIANS,
        .maximum_angle      = 170 * ROVER_DEGREES_TO_RADIANS,
    },
    [ROVER_4WS_CONFIG_SERVO_3] = {
        .timer              = BSP_PWM_USER_TIMER_SERVOS_REAR,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.1125,
        .minimum_angle      = 10 * ROVER_DEGREES_TO_RADIANS,
        .maximum_angle      = 170 * ROVER_DEGREES_TO_RADIANS,
    },
};

BspMotor_Handle_t Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_MAX] = {
    [ROVER_4WS_CONFIG_MOTOR_0] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_FRONT,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_FRONT,
            .channel = BSP_TIMER_CHANNEL_2,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [ROVER_4WS_CONFIG_MOTOR_1] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_FRONT,
            .channel = BSP_TIMER_CHANNEL_3,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_FRONT,
            .channel = BSP_TIMER_CHANNEL_4,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [ROVER_4WS_CONFIG_MOTOR_2] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_REAR,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_REAR,
            .channel = BSP_TIMER_CHANNEL_2,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [ROVER_4WS_CONFIG_MOTOR_3] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_REAR,
            .channel = BSP_TIMER_CHANNEL_3,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_MOTORS_REAR,
            .channel = BSP_TIMER_CHANNEL_4,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    }
};

BspEncoderUser_Timer_t Rover4wsConfig_Encoders[ROVER_4WS_CONFIG_MOTOR_MAX] = {
    [ROVER_4WS_CONFIG_MOTOR_0] = BSP_ENCODER_USER_TIMER_0,
    [ROVER_4WS_CONFIG_MOTOR_1] = BSP_ENCODER_USER_TIMER_1,
    [ROVER_4WS_CONFIG_MOTOR_2] = BSP_ENCODER_USER_TIMER_2,
    [ROVER_4WS_CONFIG_MOTOR_3] = BSP_ENCODER_USER_TIMER_3
};

RoverPid_Handle_t Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_MAX] = {
    [ROVER_4WS_CONFIG_MOTOR_0] = {
        .kp            = 2.0,
        .ki            = 1.5,
        .kd            = 0.000001,
        .integral      = 0.0,
        .command       = 0.0,
        .error         = 0.0,
        .output        = 0.0,
        .previous_tick = 0U,
        .enabled       = true,
        .minimum       = 0,
        .maximum       = 18.75
    },
    [ROVER_4WS_CONFIG_MOTOR_1] = {
        .kp            = 2.0,
        .ki            = 1.5,
        .kd            = 0.000001,
        .integral      = 0.0,
        .command       = 0.0,
        .error         = 0.0,
        .output        = 0.0,
        .previous_tick = 0U,
        .enabled       = true,
        .minimum       = 0,
        .maximum       = 18.75
    },
    [ROVER_4WS_CONFIG_MOTOR_2] = {
        .kp            = 2.0,
        .ki            = 1.5,
        .kd            = 0.000001,
        .integral      = 0.0,
        .command       = 0.0,
        .error         = 0.0,
        .output        = 0.0,
        .previous_tick = 0U,
        .enabled       = true,
        .minimum       = 0,
        .maximum       = 18.75
    },
    [ROVER_4WS_CONFIG_MOTOR_3] = {
        .kp            = 2.0,
        .ki            = 1.5,
        .kd            = 0.000001,
        .integral      = 0.0,
        .command       = 0.0,
        .error         = 0.0,
        .output        = 0.0,
        .previous_tick = 0U,
        .enabled       = true,
        .minimum       = 0,
        .maximum       = 18.75
    }
};

RoverPid_Handle_t Rover4wsConfig_SteeringPid = {
    .kp            = 0.0,
    .ki            = 0.0,
    .kd            = 0.0,
    .integral      = 0.0,
    .command       = 0.0,
    .error         = 0.0,
    .output        = 0.0,
    .previous_tick = 0U,
    .enabled       = true,
    .minimum       = -90 * ROVER_DEGREES_TO_RADIANS,
    .maximum       = 90 * ROVER_DEGREES_TO_RADIANS
};