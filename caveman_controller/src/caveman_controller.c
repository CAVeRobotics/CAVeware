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

    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Starting...");
    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Test log");

    HAL_Delay(10);
    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Another Test log");

    while (true)
    {
    }

    /* Temporarily instantiate test_motor to remove static analysis errors about unused motor functions */
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
    BspMotor_Stop(&test_motor);

    return 0;
}