#include "cavebot_user.h"

#include <stdbool.h>

#include "spi.h"

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio_user.h"
#include "bsp_motor.h"
#include "bsp_pwm.h"
#include "bsp_pwm_user.h"
#include "bsp_timer_user.h"
#include "bsp_servo.h"

#include "lsm6dsv16x.h"
#include "rgbw.h"

#include "cavebot.h"

static Lsm6dsv16x_Context_t kCavebotUser_Lsm6dsv16x = LSM6DSV16X_CONTEXT(&hspi2);
static bool                 CavebotUser_Armed       = false;

BspServo_Handle_t CavebotUser_Servos[CAVEBOT_USER_SERVO_MAX] = {
    [CAVEBOT_USER_SERVO_0] = {
        .timer              = BSP_PWM_USER_TIMER_2,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.115,
        .minimum_angle      = BSP_DEGREES_TO_RADIANS(10),
        .maximum_angle      = BSP_DEGREES_TO_RADIANS(170),
    },
    [CAVEBOT_USER_SERVO_1] = {
        .timer              = BSP_PWM_USER_TIMER_2,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.1125,
        .minimum_angle      = BSP_DEGREES_TO_RADIANS(10),
        .maximum_angle      = BSP_DEGREES_TO_RADIANS(170),
    },
    [CAVEBOT_USER_SERVO_2] = {
        .timer              = BSP_PWM_USER_TIMER_3,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.03,
        .maximum_duty_cycle = 0.1075,
        .minimum_angle      = BSP_DEGREES_TO_RADIANS(10),
        .maximum_angle      = BSP_DEGREES_TO_RADIANS(170),
    },
    [CAVEBOT_USER_SERVO_3] = {
        .timer              = BSP_PWM_USER_TIMER_3,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.1125,
        .minimum_angle      = BSP_DEGREES_TO_RADIANS(10),
        .maximum_angle      = BSP_DEGREES_TO_RADIANS(170),
    },
    [CAVEBOT_USER_SERVO_4] = {
        .timer              = BSP_PWM_USER_TIMER_4,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.0325,
        .maximum_duty_cycle = 0.115,
        .minimum_angle      = BSP_DEGREES_TO_RADIANS(10),
        .maximum_angle      = BSP_DEGREES_TO_RADIANS(170),
    },
    [CAVEBOT_USER_SERVO_5] = {
        .timer              = BSP_PWM_USER_TIMER_5,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 0.0972,
        .minimum_angle      = BSP_DEGREES_TO_RADIANS(0),
        .maximum_angle      = BSP_DEGREES_TO_RADIANS(130),
    },
};

BspMotor_Handle_t CavebotUser_Motors[CAVEBOT_USER_MOTOR_MAX] = {
    [CAVEBOT_USER_MOTOR_0] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_3,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_4,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [CAVEBOT_USER_MOTOR_1] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_2,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_0,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [CAVEBOT_USER_MOTOR_2] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_1,
            .channel = BSP_TIMER_CHANNEL_2,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_1,
            .channel = BSP_TIMER_CHANNEL_1,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    },
    [CAVEBOT_USER_MOTOR_3] = {
        .forward_phase = {
            .timer   = BSP_PWM_USER_TIMER_1,
            .channel = BSP_TIMER_CHANNEL_3,
        },
        .reverse_phase = {
            .timer   = BSP_PWM_USER_TIMER_1,
            .channel = BSP_TIMER_CHANNEL_4,
        },
        .minimum_duty_cycle = 0.0,
        .maximum_duty_cycle = 1.0,
        .minimum_speed      = 0.0,
        .maximum_speed      = 18.75,
        .direction          = BSP_MOTOR_DIRECTION_FORWARD,
    }
};

/* TODO move to rover_4ws.c */
CavebotPid_Handle_t CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_MAX] = {
    [CAVEBOT_USER_MOTOR_0] = {
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
    [CAVEBOT_USER_MOTOR_1] = {
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
        .maximum       = 18.75,
    },
    [CAVEBOT_USER_MOTOR_2] = {
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
        .maximum       = 18.75,
    },
    [CAVEBOT_USER_MOTOR_3] = {
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
        .maximum       = 18.75,
    }
};

BspEncoderUser_Timer_t CavebotUser_Encoders[CAVEBOT_USER_ENCODER_MAX] = {
    [CAVEBOT_USER_ENCODER_0] = BSP_ENCODER_USER_TIMER_0,
    [CAVEBOT_USER_ENCODER_1] = BSP_ENCODER_USER_TIMER_1,
    [CAVEBOT_USER_ENCODER_2] = BSP_ENCODER_USER_TIMER_2,
    [CAVEBOT_USER_ENCODER_3] = BSP_ENCODER_USER_TIMER_3
};

A9488_Context_t CavebotUser_StepperMotor = {
    .timer       = BSP_TIMER_USER_TIMER_0,
    .step        = BSP_GPIO_USER_PIN_STEPPER_MOTOR_STEP,
    .direction   = BSP_GPIO_USER_PIN_STEPPER_MOTOR_DIRECTION,
    .half_pulses = 0U,
};

Rgbw_Handle_t CavebotUser_Rgbw = {
    .pins = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_USER_PIN_STATUS_LED_RED,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_USER_PIN_STATUS_LED_GREEN,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_USER_PIN_STATUS_LED_BLUE,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_USER_PIN_STATUS_LED_WHITE,
    },
};

Accelerometer_Handle_t CavebotUser_Accelerometer = LSM6DSV16X_ACCELEROMETER_HANDLE(kCavebotUser_Lsm6dsv16x);
Gyroscope_Handle_t     CavebotUser_Gyroscope     = LSM6DSV16X_GYROSCOPE_HANDLE(kCavebotUser_Lsm6dsv16x);

Cavebot_Error_t CavebotUser_Initialize(void)
{
    Bsp_Error_t error = Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_YELLOW);

    if (BSP_ERROR_NONE == error)
    {
        error = Accelerometer_Initialize(&CavebotUser_Accelerometer);
    }

    if (BSP_ERROR_NONE == error)
    {
        error = Gyroscope_Initialize(&CavebotUser_Gyroscope);
    }

    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_0);
    }

    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_1);
    }

    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_2);
    }

    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Start(BSP_ENCODER_USER_TIMER_3);
    }

    if (BSP_ERROR_NONE == error)
    {
        CavebotUser_Armed = Cavebot_IsArmed();

        /* TODO make sound asynchronous, error handling? */
        /* Initialization sound */
        BspPwm_Start(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
        BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1, 0.5);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_6, 31110);
        Bsp_Delay(100);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_6, 23333);
        Bsp_Delay(100);
        BspPwm_SetPeriod(BSP_PWM_USER_TIMER_6, 15556);
        Bsp_Delay(100);
        BspPwm_Stop(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
    }

    return Cavebot_BspToCavebotError(error);
}

Cavebot_Error_t CavebotUser_SensorTask(void)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    /* TODO sample IMU*/
    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_0);
    }
    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_1);
    }
    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_2);
    }
    if (BSP_ERROR_NONE == error)
    {
        error = BspEncoder_Sample(BSP_ENCODER_USER_TIMER_3);
    }

    return Cavebot_BspToCavebotError(error);
}

Cavebot_Error_t CavebotUser_Task(void)
{
    /* TODO error handling? */

    const bool armed = Cavebot_IsArmed();
    if (armed != CavebotUser_Armed)
    {
        CavebotUser_Armed = armed;

        if (armed)
        {
            /* Arm sound */
            BspPwm_Start(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
            BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1, 0.5);
            BspPwm_SetPeriod(BSP_PWM_USER_TIMER_6, 15556);
            Bsp_Delay(100);
            BspPwm_Stop(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
            Bsp_Delay(100);
            BspPwm_Start(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
            Bsp_Delay(100);
            BspPwm_Stop(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
        }
        else
        {
            /* Disarm sound */
            BspPwm_Start(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
            BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1, 0.5);
            BspPwm_SetPeriod(BSP_PWM_USER_TIMER_6, 15556);
            Bsp_Delay(300);
            BspPwm_Stop(BSP_PWM_USER_TIMER_6, BSP_TIMER_CHANNEL_1);
        }
    }

    if (armed)
    {
        Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_RED);
    }
    else
    {
        Rgbw_SetColor(&CavebotUser_Rgbw, RGBW_COLOR_GREEN);
    }

    return CAVEBOT_ERROR_NONE;
}