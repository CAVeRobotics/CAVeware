#include "rover.h"

#include "bsp.h"
#include "bsp_logger.h"

#include "rover_4ws.h"
#include "rover_camera.h"

static Rover_Mode_t Rover_Mode    = ROVER_MODE_STARTUP;
static const char*  kRover_LogTag = "ROVER";

static Rover_Error_t Rover_SetModeStartup(void);
static Rover_Error_t Rover_SetModeConfigure(void);
static Rover_Error_t Rover_SetModeRun(void);

void Rover_Initialize(void)
{
    (void)Rover_SetMode(ROVER_MODE_STARTUP);
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

Rover_Mode_t Rover_GetMode(void)
{
    return Rover_Mode;
}

Rover_Error_t Rover_SetMode(const Rover_Mode_t mode)
{
    Rover_Error_t error = ROVER_ERROR_MODE;

    switch (mode)
    {
    case ROVER_MODE_STARTUP:
        error = Rover_SetModeStartup();
        break;
    case ROVER_MODE_CONFIGURE:
        error = Rover_SetModeConfigure();
        break;
    case ROVER_MODE_RUN:
        error = Rover_SetModeRun();
        break;
    default:
        break;
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kRover_LogTag, "Failed to set mode %d with error %d", (int)mode, (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kRover_LogTag, "Set mode %d", (int)mode);
    }

    return error;
}

static Rover_Error_t Rover_SetModeStartup(void)
{
    Rover_Mode = ROVER_MODE_STARTUP;

    /* TODO */

    (void)Rover4ws_StopMotors();
    (void)Rover4ws_DisableSteering();
    (void)RoverCamera_Disable();

    return ROVER_ERROR_NONE;
}

static Rover_Error_t Rover_SetModeConfigure(void)
{
    Rover_Mode = ROVER_MODE_CONFIGURE;

    /* TODO */

    (void)Rover4ws_StopMotors();
    (void)Rover4ws_DisableSteering();
    (void)RoverCamera_Disable();

    return ROVER_ERROR_NONE;
}

static Rover_Error_t Rover_SetModeRun(void)
{
    Rover_Mode = ROVER_MODE_RUN;

    /* TODO */

    (void)Rover4ws_EnableSteering();
    (void)Rover4ws_StartMotors();
    (void)RoverCamera_Enable();

    return ROVER_ERROR_NONE;
}