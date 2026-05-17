#include "pid.h"

#include <stdbool.h>

#include "bsp.h"
#include "bsp_tick.h"

#include "cavebot.h"

Cavebot_Error_t Pid_Reset(Pid_Handle_t *const handle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != handle)
    {
        handle->integral         = 0.0;
        handle->command          = 0.0;
        handle->error            = 0.0;
        handle->output           = 0.0;
        handle->previous_tick    = BspTick_GetMicroseconds();
        handle->integral_enabled = true;

        error = CAVEBOT_ERROR_NONE;
    }

    return error;
}

Cavebot_Error_t Pid_Enable(Pid_Handle_t *const handle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != handle)
    {
        (void)Pid_Reset(handle);
        handle->enabled = true;

        error = CAVEBOT_ERROR_NONE;
    }

    return error;
}

Cavebot_Error_t Pid_Disable(Pid_Handle_t *const handle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != handle)
    {
        handle->enabled = false;

        error = CAVEBOT_ERROR_NONE;
    }

    return error;
}

Cavebot_Error_t Pid_Update(Pid_Handle_t *const handle, const double actual)
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
        const Bsp_Microsecond_t tick       = BspTick_GetMicroseconds();
        const Bsp_Second_t      delta_time = BspTick_GetElapsedMicroseconds(handle->previous_tick, tick);
        const double            pid_error  = handle->command - actual;
        double                  derivative = 0.0;
        if (delta_time > 0.0)
        {
            derivative = (pid_error - handle->error) / delta_time;
        }

        if (!handle->integral_enabled && !Bsp_CompareDoubleSigns(&handle->error, &pid_error))
        {
            handle->integral_enabled = true;
        }

        if (handle->integral_enabled)
        {
            handle->integral += pid_error * delta_time;
        }

        const double output           = (handle->kp * pid_error) + (handle->ki * handle->integral) + (handle->kd * derivative) + (handle->kff * handle->command);
        const double delta_output     = output - handle->output;
        const double max_delta_output = handle->rate_limit * delta_time;
        if ((delta_output > max_delta_output) && (handle->command > 0.0))
        {
            handle->output += max_delta_output;
        }
        else if ((delta_output < -max_delta_output) && (handle->command < 0.0))
        {
            handle->output -= max_delta_output;
        }
        else
        {
            handle->output = output;
        }

        if (handle->output >= handle->maximum)
        {
            handle->output           = handle->maximum;
            handle->integral_enabled = false;
        }
        else if (handle->output <= handle->minimum)
        {
            handle->output           = handle->minimum;
            handle->integral_enabled = false;
        }

        handle->error         = pid_error;
        handle->previous_tick = tick;
    }

    return error;
}