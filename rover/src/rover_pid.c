#include "rover_pid.h"

#include "bsp_tick.h"

#include "rover.h"

Rover_Error_t RoverPid_Reset(RoverPid_Handle_t *const handle)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != handle)
    {
        handle->integral      = 0.0;
        handle->command       = 0.0;
        handle->error         = 0.0;
        handle->output        = 0.0;
        handle->previous_tick = 0U;

        error = ROVER_ERROR_NONE;
    }

    return error;
}

Rover_Error_t RoverPid_Enable(RoverPid_Handle_t *const handle)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != handle)
    {
        (void)RoverPid_Reset(handle);
        handle->enabled = true;

        error = ROVER_ERROR_NONE;
    }

    return error;
}

Rover_Error_t RoverPid_Disable(RoverPid_Handle_t *const handle)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != handle)
    {
        handle->enabled = false;

        error = ROVER_ERROR_NONE;
    }

    return error;
}

Rover_Error_t RoverPid_Update(RoverPid_Handle_t *const handle, const double actual, const Rover_Microsecond_t tick)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (NULL == handle)
    {
        error = ROVER_ERROR_NULL;
    }
    else if (!handle->enabled)
    {
        handle->output = handle->command;
    }
    else
    {
        double delta      = (double)(tick - handle->previous_tick) / BSP_TICK_MICROSECONDS_PER_SECOND;
        double pid_error  = handle->command - actual;
        double derivative = (pid_error - handle->error) / delta;

        handle->integral     += pid_error * delta;
        handle->output        = (handle->kp * pid_error) + (handle->ki * handle->integral) + (handle->kd * derivative);
        handle->error         = pid_error;
        handle->previous_tick = tick;

        if (handle->output > handle->maximum)
        {
            handle->output = handle->maximum;
        }
        else if (handle->output < handle->minimum)
        {
            handle->output = handle->minimum;
        }
    }

    return error;
}