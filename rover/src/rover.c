#include "rover.h"

#include <stdbool.h>

#include "bsp.h"
#include "bsp_logger.h"

#include "rover_4ws.h"
#include "rover_camera.h"

static const char* kRover_LogTag = "ROVER";
static bool        Rover_Armed   = false;

void Rover_Initialize(void)
{
    (void)Rover_Dearm();
    (void)Rover4ws_EnableEncoders();
}

void Rover_Task(void)
{
    /* TODO tasks that need to occur repeatedly, e.g. feedback control */
    (void)Rover4ws_SampleEncoders();
}

Rover_Error_t Rover_BspToRoverError(const Bsp_Error_t bsp_error)
{
    Rover_Error_t rover_error = ROVER_ERROR_NONE;

    switch (bsp_error)
    {
    case BSP_ERROR_NONE:
        break;
    case BSP_ERROR_NULL:
        rover_error = ROVER_ERROR_NULL;
        break;
    case BSP_ERROR_PERIPHERAL:
        rover_error = ROVER_ERROR_PERIPHERAL;
        break;
    case BSP_ERROR_HAL:
    case BSP_ERROR_BUSY:
    case BSP_ERROR_TIMEOUT:
    case BSP_ERROR_VALUE:
    default:
        rover_error = ROVER_ERROR_BSP;
        break;
    }

    return rover_error;
}

Rover_Error_t Rover_Arm(void)
{
    Rover_Error_t error = Rover4ws_EnableSteering();

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover4ws_StartMotors();
    }

    if (ROVER_ERROR_NONE == error)
    {
        error = RoverCamera_Enable();
    }

    if (ROVER_ERROR_NONE == error)
    {
        BSP_LOGGER_LOG_INFO(kRover_LogTag, "Armed", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kRover_LogTag, "Failed to arm with error %d", (int)error);
    }

    return error;
}

Rover_Error_t Rover_Dearm(void)
{
    Rover_Error_t error = Rover4ws_DisableSteering();

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover4ws_StopMotors();
    }

    if (ROVER_ERROR_NONE == error)
    {
        error = RoverCamera_Disable();
    }

    if (ROVER_ERROR_NONE == error)
    {
        BSP_LOGGER_LOG_INFO(kRover_LogTag, "Dearmed", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kRover_LogTag, "Failed to dearm with error %d", (int)error);
    }

    return error;
}

bool Rover_IsArmed(void)
{
    return Rover_Armed;
}

Rover_Error_t Rover_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate)
{
    return Rover4ws_Drive(speed, turn_rate);
}