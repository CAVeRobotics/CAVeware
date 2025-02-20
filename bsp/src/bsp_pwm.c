#include "bsp_pwm.h"

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_pwm_user.h"

#define BSP_PWM_MAX_PERIOD (Bsp_Microsecond_t)(UINT16_MAX)

static inline bool BspPwm_IsValidTimerChannel(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel);

Bsp_Error_t BspPwm_Start(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (!BspPwm_IsValidTimerChannel(timer, channel))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        error = (Bsp_Error_t)HAL_TIM_PWM_Start(BspPwmUser_TimerConfigTable[timer].timer_handle, channel);
    }

    return error;
}

Bsp_Error_t BspPwm_Stop(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (!BspPwm_IsValidTimerChannel(timer, channel))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        error = (Bsp_Error_t)HAL_TIM_PWM_Stop(BspPwmUser_TimerConfigTable[timer].timer_handle, channel);
    }

    return error;
}

Bsp_Error_t BspPwm_SetPeriod(const BspPwmUser_Timer_t timer, const Bsp_Microsecond_t period)
{
    Bsp_Error_t       error           = BSP_ERROR_NONE;
    Bsp_Microsecond_t period_adjusted = period - 1;

    if (timer >= BSP_PWM_USER_TIMER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else if (period_adjusted > BSP_PWM_MAX_PERIOD)
    {
        error = BSP_ERROR_VALUE;
    }
    else
    {
        BspPwmUser_TimerConfigTable[timer].timer_handle->Instance->ARR = (uint16_t)(period_adjusted);
    }

    return error;
}

Bsp_Error_t BspPwm_SetDutyCycle(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel, const Bsp_Percent_t duty_cycle)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (!BspPwm_IsValidTimerChannel(timer, channel))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else if ((duty_cycle < 0) | (duty_cycle > 1.0))
    {
        error = BSP_ERROR_VALUE;
    }
    else
    {
        TIM_TypeDef *timer_instance  = BspPwmUser_TimerConfigTable[timer].timer_handle->Instance;
        uint32_t     capture_compare = (uint32_t)(timer_instance->ARR * duty_cycle);

        switch (channel)
        {
        case BSP_TIMER_CHANNEL_1:
            timer_instance->CCR1 = capture_compare;
            break;
        case BSP_TIMER_CHANNEL_2:
            timer_instance->CCR2 = capture_compare;
            break;
        case BSP_TIMER_CHANNEL_3:
            timer_instance->CCR3 = capture_compare;
            break;
        case BSP_TIMER_CHANNEL_4:
            timer_instance->CCR4 = capture_compare;
            break;
        default:
            error = BSP_ERROR_PERIPHERAL;
            break;
        }
    }

    return error;
}

static inline bool BspPwm_IsValidTimerChannel(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel)
{
    bool valid = true;

    if ((timer >= BSP_PWM_USER_TIMER_MAX) || (channel > BspPwmUser_TimerConfigTable[timer].max_channel))
    {
        valid = false;
    }

    return valid;
}