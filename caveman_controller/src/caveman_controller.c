#include "caveman_controller.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_logger.h"
#include "bsp_motor.h"
#include "bsp_pwm.h"
#include "bsp_pwm_user.h"
#include "bsp_servo.h"
#include "bsp_tick.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

static const char *kCaveman_LogTag = "CAVEMAN";

int main(void)
{
    Bsp_Initialize();
    BspTick_Start();
    BspEncoder_Start(BSP_ENCODER_USER_TIMER_0);

    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Starting...");

    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "%u", (uint32_t)BspTick_GetMicroseconds());

    BspServo_Handle_t wheel_0_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0.025,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 180,
    };
    BspServo_Handle_t wheel_1_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0.025,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 180,
    };
    BspServo_Handle_t wheel_2_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_3,
        .minimum_duty_cycle = 0.025,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 180,
    };
    BspServo_Handle_t wheel_3_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_4,
        .minimum_duty_cycle = 0.025,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 180,
    };
    BspMotor_Handle_t test_motor = {
        .forward_phase.timer   = BSP_PWM_USER_TIMER_MOTORS,
        .forward_phase.channel = BSP_TIMER_CHANNEL_1,
        .reverse_phase.timer   = BSP_PWM_USER_TIMER_MOTORS,
        .reverse_phase.channel = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle    = 0.0,
        .maximum_duty_cycle    = 1.0,
        .minimum_speed         = 0.0,
        .maximum_speed         = 3.14159265, /* TODO determine max motor speed under load */
        .direction             = BSP_MOTOR_DIRECTION_FORWARD,
    };

    BspMotor_Start(&test_motor);
    BspMotor_SetDirection(&test_motor, BSP_MOTOR_DIRECTION_FORWARD);
    BspMotor_SetDutyCycle(&test_motor, 0.0);

    HAL_Delay(2000);

    BspPwm_Start(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_1);
    BspPwm_Start(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_2);
    BspPwm_Start(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_3);
    BspPwm_Start(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_4);
    BspServo_Start(&wheel_0_servo);
    BspServo_Start(&wheel_1_servo);
    BspServo_Start(&wheel_2_servo);
    BspServo_Start(&wheel_3_servo);

    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_1, 0.10);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_2, 0.10);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_3, 0.10);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_4, 0.10);
    BspServo_SetAngle(&wheel_0_servo, 0);
    BspServo_SetAngle(&wheel_1_servo, 0);
    BspServo_SetAngle(&wheel_2_servo, 0);
    BspServo_SetAngle(&wheel_3_servo, 0);

    HAL_Delay(3000);

    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_1, 0.50);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_2, 0.50);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_3, 0.50);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_4, 0.50);
    BspServo_SetAngle(&wheel_0_servo, 60);
    BspServo_SetAngle(&wheel_1_servo, 60);
    BspServo_SetAngle(&wheel_2_servo, 60);
    BspServo_SetAngle(&wheel_3_servo, 60);

    HAL_Delay(3000);

    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_1, 1.0);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_2, 1.0);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_3, 1.0);
    BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_4, 1.0);
    BspServo_SetAngle(&wheel_0_servo, 120);
    BspServo_SetAngle(&wheel_1_servo, 120);
    BspServo_SetAngle(&wheel_2_servo, 120);
    BspServo_SetAngle(&wheel_3_servo, 120);

    HAL_Delay(3000);

    BspPwm_Stop(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_1);
    BspPwm_Stop(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_2);
    BspPwm_Stop(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_3);
    BspPwm_Stop(BSP_PWM_USER_TIMER_MOTORS, BSP_TIMER_CHANNEL_4);

    BspServo_Stop(&wheel_0_servo);
    BspServo_Stop(&wheel_1_servo);
    BspServo_Stop(&wheel_2_servo);
    BspServo_Stop(&wheel_3_servo);

    BspEncoderUser_Pulse_t pulses = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses;
    while (true)
    {
        bool updated = false;

        BspEncoder_Sample(BSP_ENCODER_USER_TIMER_0);

        if (BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses != pulses)
        {
            pulses  = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses;
            updated = true;
        }

        if (updated)
        {
            BSP_LOGGER_LOG_INFO(kCaveman_LogTag, "Pulses: %d", (int32_t)pulses);
        }

        HAL_Delay(1000);
    }

    return 0;
}