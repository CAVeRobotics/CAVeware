#include "caveman_controller.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_logger.h"
#include "bsp_pwm_user.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

static const char *kCaveman_LogTag = "CAVEMAN";

int main(void)
{
    Bsp_Initialize();

    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Starting...");

    BspServo_Handle_t wheel_0_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_1,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120,
    };
    BspServo_Handle_t wheel_1_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_2,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120,
    };
    BspServo_Handle_t wheel_2_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_3,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120,
    };
    BspServo_Handle_t wheel_3_servo = {
        .timer              = BSP_PWM_USER_TIMER_STEERING_SERVOS,
        .channel            = BSP_TIMER_CHANNEL_4,
        .minimum_duty_cycle = 0,
        .maximum_duty_cycle = 0.125,
        .minimum_angle      = 0,
        .maximum_angle      = 120,
    };

    BspServo_Start(&wheel_0_servo);
    BspServo_Start(&wheel_1_servo);
    BspServo_Start(&wheel_2_servo);
    BspServo_Start(&wheel_3_servo);

    // BspServo_SetAngle(&wheel_0_servo, 0);
    // BspServo_SetAngle(&wheel_1_servo, 0);
    // BspServo_SetAngle(&wheel_2_servo, 0);
    // BspServo_SetAngle(&wheel_3_servo, 0);

    BspServo_SetAngle(&wheel_0_servo, 60);
    // HAL_Delay(3000);
    BspServo_SetAngle(&wheel_1_servo, 60);
    // HAL_Delay(3000);
    BspServo_SetAngle(&wheel_2_servo, 60);
    // HAL_Delay(3000);
    BspServo_SetAngle(&wheel_3_servo, 60);

    // HAL_Delay(1000);

    // BspServo_SetAngle(&wheel_0_servo, 120);
    // BspServo_SetAngle(&wheel_1_servo, 120);
    // BspServo_SetAngle(&wheel_2_servo, 120);
    // BspServo_SetAngle(&wheel_3_servo, 120);

    // BspPwm_Start(BSP_PWM_USER_TIMER_STEERING_SERVOS, BSP_TIMER_CHANNEL_1);
    // BspPwm_SetDutyCycle(BSP_PWM_USER_TIMER_STEERING_SERVOS, BSP_TIMER_CHANNEL_1, 0.075);

    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Done");

    return 0;
}