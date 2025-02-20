#include "bsp_motor.h"

#include "bsp.h"
#include "bsp_pwm.h"

static Bsp_Error_t BspMotor_SetDutyCycleUnlimited(BspMotor_Handle_t *const handle, const Bsp_Percent_t duty_cycle);

Bsp_Error_t BspMotor_Start(BspMotor_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspPwm_Start(handle->forward_phase.timer, handle->forward_phase.channel);
        Bsp_Error_t reverse_start_error = BspPwm_Start(handle->reverse_phase.timer, handle->reverse_phase.channel);

        Bsp_Error_t forward_duty_cycle_error = BspPwm_SetDutyCycle(handle->forward_phase.timer, handle->forward_phase.channel, 0.0);
        Bsp_Error_t reverse_duty_cycle_error = BspPwm_SetDutyCycle(handle->reverse_phase.timer, handle->reverse_phase.channel, 0.0);

        if (BSP_ERROR_NONE != error)
        {
        }
        else if (BSP_ERROR_NONE != reverse_start_error)
        {
            error = reverse_start_error;
        }
        else if (BSP_ERROR_NONE != forward_duty_cycle_error)
        {
            error = forward_duty_cycle_error;
        }
        else
        {
            error = reverse_duty_cycle_error;
        }
    }

    return error;
}

Bsp_Error_t BspMotor_Stop(BspMotor_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspPwm_Stop(handle->forward_phase.timer, handle->forward_phase.channel);
        Bsp_Error_t reverse_stop_error = BspPwm_Stop(handle->reverse_phase.timer, handle->reverse_phase.channel);

        if (BSP_ERROR_NONE != error)
        {
            error = reverse_stop_error;
        }
    }

    return error;
}

Bsp_Error_t BspMotor_Forward(BspMotor_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspMotor_SetDirection(handle, BSP_MOTOR_DIRECTION_FORWARD);
    }

    return error;
}

Bsp_Error_t BspMotor_Reverse(BspMotor_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspMotor_SetDirection(handle, BSP_MOTOR_DIRECTION_REVERSE);
    }

    return error;
}

Bsp_Error_t BspMotor_SetDirection(BspMotor_Handle_t *const handle, const BspMotor_Direction_t direction)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        if (direction != handle->direction)
        {
            BspMotor_SetDutyCycleUnlimited(handle, 0.0);

            /* TODO SD-220 insert dead time on transition */
        }

        handle->direction = direction;
    }

    return error;
}

Bsp_Error_t BspMotor_SetDutyCycle(BspMotor_Handle_t *const handle, const Bsp_Percent_t duty_cycle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == handle)
    {
    }
    else if ((duty_cycle > handle->maximum_duty_cycle) || (duty_cycle < handle->minimum_duty_cycle))
    {
        error = BSP_ERROR_VALUE;
    }
    else
    {
        error = BspMotor_SetDutyCycleUnlimited(handle, duty_cycle);
    }

    return error;
}

Bsp_Error_t BspMotor_SetSpeed(BspMotor_Handle_t *const handle, const Bsp_RadiansPerSecond_t speed)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == handle)
    {
    }
    else if ((speed > handle->maximum_speed) || (speed < handle->minimum_speed))
    {
        error = BSP_ERROR_VALUE;
    }
    else
    {
        error = BspMotor_SetDutyCycle(handle, Bsp_Map(speed, handle->minimum_speed, handle->maximum_speed, handle->minimum_duty_cycle, handle->maximum_duty_cycle));
    }

    return error;
}

Bsp_Error_t BspMotor_Brake(BspMotor_Handle_t *const handle)
{
    /* TODO SD-219 verify that duty cycle of one produces high output */
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspPwm_SetDutyCycle(handle->forward_phase.timer, handle->forward_phase.channel, 1.0);

        if (BSP_ERROR_NONE == error)
        {
            error = BspPwm_SetDutyCycle(handle->reverse_phase.timer, handle->reverse_phase.channel, 1.0);
        }
    }

    return error;
}

Bsp_Error_t BspMotor_Coast(BspMotor_Handle_t *const handle)
{
    /* TODO SD-219 verify that duty cycle of zero produces zero output */
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspPwm_SetDutyCycle(handle->forward_phase.timer, handle->forward_phase.channel, 0);

        if (BSP_ERROR_NONE == error)
        {
            error = BspPwm_SetDutyCycle(handle->reverse_phase.timer, handle->reverse_phase.channel, 0);
        }
    }

    return error;
}

static Bsp_Error_t BspMotor_SetDutyCycleUnlimited(BspMotor_Handle_t *const handle, const Bsp_Percent_t duty_cycle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == handle)
    {
    }
    else if (BSP_MOTOR_DIRECTION_FORWARD == handle->direction)
    {
        error = BspPwm_SetDutyCycle(handle->forward_phase.timer, handle->forward_phase.channel, duty_cycle);
    }
    else
    {
        error = BspPwm_SetDutyCycle(handle->reverse_phase.timer, handle->reverse_phase.channel, duty_cycle);
    }

    return error;
}