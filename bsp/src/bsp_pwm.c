#include "bsp_pwm.h"

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "bsp_pwm_user.h"
#include "bsp_types.h"

#define BSP_PWM_MAX_PERIOD (BspTypes_Microsecond_t)(UINT16_MAX)

extern BspPwmUser_TimerConfig_t BspPwmUser_TimerConfigTable[BSP_PWM_USER_TIMER_MAX];

static inline bool BspPwm_IsValidTimerChannel(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel);

BspTypes_Error_t BspPwm_Start(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel)
{
    BspTypes_Error_t error = BSP_TYPES_ERROR_NONE;

    if (!BspPwm_IsValidTimerChannel(timer, channel))
    {
        error = BSP_TYPES_ERROR_PERIPHERAL;
    }
    else
    {
        error = (BspTypes_Error_t)HAL_TIM_PWM_Start(BspPwmUser_TimerConfigTable[timer].timer_handle, channel);
    }

    return error;
}

BspTypes_Error_t BspPwm_Stop(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel)
{
    BspTypes_Error_t error = BSP_TYPES_ERROR_NONE;

    if (!BspPwm_IsValidTimerChannel(timer, channel))
    {
        error = BSP_TYPES_ERROR_PERIPHERAL;
    }
    else
    {
        error = (BspTypes_Error_t)HAL_TIM_PWM_Stop(BspPwmUser_TimerConfigTable[timer].timer_handle, channel);
    }

    return error;
}

BspTypes_Error_t BspPwm_SetPeriod(const BspPwmUser_Timer_t timer, const BspTypes_Microsecond_t period)
{
    BspTypes_Error_t error = BSP_TYPES_ERROR_NONE;
    BspTypes_Microsecond_t period_adjusted = period - 1;

    if (timer >= BSP_PWM_USER_TIMER_MAX)
    {
        error = BSP_TYPES_ERROR_PERIPHERAL;
    }
    else if (period_adjusted > BSP_PWM_MAX_PERIOD)
    {
        error = BSP_TYPES_ERROR_VALUE;
    }
    else
    {
        BspPwmUser_TimerConfigTable[timer].timer_handle->Instance->ARR = (uint16_t)(period_adjusted);
    }

    return error;
}

BspTypes_Error_t BspPwm_SetDutyCycle(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel, const BspTypes_Percent_t duty_cycle)
{
    BspTypes_Error_t error = BSP_TYPES_ERROR_NONE;

    if (!BspPwm_IsValidTimerChannel(timer, channel))
    {
        error = BSP_TYPES_ERROR_PERIPHERAL;
    }
    else if ((duty_cycle < 0) | (duty_cycle > 1.0))
    {
        error = BSP_TYPES_ERROR_VALUE;
    }
    else
    {
        TIM_TypeDef *timer_instance = BspPwmUser_TimerConfigTable[timer].timer_handle->Instance;
        uint32_t capture_compare = (uint32_t)(timer_instance->ARR * duty_cycle);

        switch (channel)
        {
        case BSP_TYPES_TIMER_CHANNEL_1:
            timer_instance->CCR1 = capture_compare;
            break;
        case BSP_TYPES_TIMER_CHANNEL_2:
            timer_instance->CCR2 = capture_compare;
            break;
        case BSP_TYPES_TIMER_CHANNEL_3:
            timer_instance->CCR3 = capture_compare;
            break;
        case BSP_TYPES_TIMER_CHANNEL_4:
            timer_instance->CCR4 = capture_compare;
            break;
        default:
            error = BSP_TYPES_ERROR_PERIPHERAL;
            break;
        }
    }

    return error;
}

static inline bool BspPwm_IsValidTimerChannel(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel)
{
    bool valid = true;

    if ((timer >= BSP_PWM_USER_TIMER_MAX) || (channel > BspPwmUser_TimerConfigTable[timer].max_channel))
    {
        valid = false;
    }

    return valid;
}