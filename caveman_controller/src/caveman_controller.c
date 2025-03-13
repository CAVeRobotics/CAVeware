#include "caveman_controller.h"

#include <stdbool.h>

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

#include "caveman_cavetalk.h"
#include "rover.h"

static const char *kCaveman_LogTag = "CAVEMAN";

static void Caveman_Initialize(void);

int main(void)
{
    Caveman_Initialize();
    Rover_SetMode(ROVER_MODE_RUN);

    while (true)
    {
        Rover_Task();
        CavemanCaveTalk_Task();
    }

    return 0;
}

static void Caveman_Initialize(void)
{
    Bsp_Initialize();
    if (BSP_ERROR_NONE != BspTick_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCaveman_LogTag, "Failed to start BSP Tick");
    }
    Rover_Initialize();
    if (CAVE_TALK_ERROR_NONE != CavemanCaveTalk_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCaveman_LogTag, "Failed to start Rover");
    }
    BSP_LOGGER_LOG_DEBUG(kCaveman_LogTag, "Initialized");

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); /* Test IMU LED */
}