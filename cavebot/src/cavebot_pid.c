#include "cavebot_pid.h"

#include "bsp_tick.h"

#include "cavebot.h"

Cavebot_Error_t CavebotPid_Reset(CavebotPid_Handle_t *const handle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != handle)
    {
        handle->integral      = 0.0;
        handle->command       = 0.0;
        handle->error         = 0.0;
        handle->output        = 0.0;
        handle->previous_tick = 0U;

        error = CAVEBOT_ERROR_NONE;
    }

    return error;
}

Cavebot_Error_t CavebotPid_Enable(CavebotPid_Handle_t *const handle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != handle)
    {
        (void)CavebotPid_Reset(handle);
        handle->enabled = true;

        error = CAVEBOT_ERROR_NONE;
    }

    return error;
}

Cavebot_Error_t CavebotPid_Disable(CavebotPid_Handle_t *const handle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != handle)
    {
        handle->enabled = false;

        error = CAVEBOT_ERROR_NONE;
    }

    return error;
}

Cavebot_Error_t CavebotPid_Update(CavebotPid_Handle_t *const handle, const double actual, const Bsp_Microsecond_t tick)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (NULL == handle)
    {
        error = CAVEBOT_ERROR_NULL;
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