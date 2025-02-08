#include "caveman_controller.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_pwm_user.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

// #define CAVEMAN_PRINTBUFFER_SIZE 1024UL

// static uint8_t Caveman_PrintBuffer[CAVEMAN_PRINTBUFFER_SIZE];

// static void Caveman_Log(const char *const format, ...);

int main(void)
{
    Bsp_Initialize();

    // Caveman_Log("Starting CAVEMAN...\r\n");

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

    // Caveman_Log("CAVEMAN done\r\n");
    // HAL_UART_Transmit(&huart2, (uint8_t *)"CAVEMAN done\r\n", 15, HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart2, (uint8_t *)"foobar\r\n", 9, HAL_MAX_DELAY);

    BspUart_Transmit(BSP_UART_USER_LOG, (uint8_t *)"CAVEMAN done\r\n", 15);
    BspUart_Transmit(BSP_UART_USER_LOG, (uint8_t *)"foobar\r\n", 9);

    return 0;
}

// static void Caveman_Log(const char *const format, ...)
// {
//     if (format != NULL)
//     {
//         va_list args;

//         va_start(args, format);
//         vsnprintf((char *)Caveman_PrintBuffer, CAVEMAN_PRINTBUFFER_SIZE, format, args);
//         va_end(args);

//         HAL_UART_Transmit(&huart2, Caveman_PrintBuffer, strlen((char *)Caveman_PrintBuffer), HAL_MAX_DELAY);
//     }
// }