#include "bsp_servo.h"

#include "bsp.h"
#include "bsp_pwm.h"

Bsp_Error_t BspServo_Start(BspServo_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspPwm_Start(handle->timer, handle->channel);
    }

    return error;
}

Bsp_Error_t BspServo_Stop(BspServo_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = BspPwm_Stop(handle->timer, handle->channel);
    }

    return error;
}

Bsp_Error_t BspServo_SetDutyCycle(BspServo_Handle_t *const handle, const Bsp_Percent_t duty_cycle)
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
        error = BspPwm_SetDutyCycle(handle->timer, handle->channel, duty_cycle);
    }

    return error;
}

Bsp_Error_t BspServo_SetAngle(BspServo_Handle_t *const handle, const Bsp_Radian_t angle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == handle)
    {
    }
    else if ((angle > handle->maximum_angle) || (angle < handle->minimum_angle))
    {
        error = BSP_ERROR_VALUE;
    }
    else
    {
        error = BspPwm_SetDutyCycle(handle->timer,
                                    handle->channel,
                                    Bsp_Map(angle, handle->minimum_angle, handle->maximum_angle, handle->minimum_duty_cycle, handle->maximum_duty_cycle));
    }

    return error;
}