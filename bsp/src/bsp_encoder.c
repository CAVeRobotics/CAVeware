#include "bsp_encoder.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_encoder_user.h"
#include "bsp_tick.h"

#define BSP_ENCODER_RADIANS_PER_ROTATION (2 * BSP_PI)
#define BSP_ENCODER_PHASES               (Bsp_EncoderPulse_t)2

extern void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *tim_encoderHandle);

static void BspEncoder_SamplePulses(Bsp_Encoder_t *const handle);
static void BspEncoder_TimerCallback(const Bsp_TimerHandle_t *const handle);
static inline void BspEncoder_TimerCallbackHandler(Bsp_Encoder_t *const handle);
static Bsp_Encoder_t *BspEncoder_GetEncoderHandle(const Bsp_TimerHandle_t *const timer_handle);

Bsp_Error_t BspEncoder_Start(const BspEncoderUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (timer >= BSP_ENCODER_USER_TIMER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else if (NULL == BspEncoderUser_HandleTable[timer].timer_handle)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        Bsp_Encoder_t *handle = &BspEncoderUser_HandleTable[timer];

        switch (handle->mode)
        {
        case BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON:
            handle->mode              = BSP_ENCODER_USER_MODE_RADIANS_PER_PULSE;
            handle->radians_per_pulse = BSP_ENCODER_RADIANS_PER_ROTATION / handle->pulses_per_rotation;
            break;
        case BSP_ENCODER_USER_MODE_RADIANS_PER_PULSE:
        default:
            break;
        }

        Bsp_TimerHandle_t *timer_handle = handle->timer_handle;
        timer_handle->PeriodElapsedCallback = (void (*)(Bsp_TimerHandle_t *)) BspEncoder_TimerCallback;

        handle->pulses_per_period        = (Bsp_EncoderPulse_t)((Bsp_EncoderPulse_t)__HAL_TIM_GET_AUTORELOAD(timer_handle) + (Bsp_EncoderPulse_t)1);
        handle->sampling                 = false;
        handle->pulse_offset             = 0U;
        handle->previous_periods_elapsed = 0;
        handle->periods_elapsed          = 0;
        handle->time                     = BspTick_GetMicroseconds();
        handle->pulses                   = 0;
        handle->raw_angular_rate         = 0;
        handle->angular_rate             = 0;

        HAL_TIM_Encoder_MspInit(timer_handle);
        error = (Bsp_Error_t)HAL_TIM_Base_Start_IT(timer_handle);
    }

    return error;
}

Bsp_Error_t BspEncoder_Stop(const BspEncoderUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (timer >= BSP_ENCODER_USER_TIMER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else if (NULL == BspEncoderUser_HandleTable[timer].timer_handle)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        error = (Bsp_Error_t)HAL_TIM_Base_Stop_IT(BspEncoderUser_HandleTable[timer].timer_handle);
        HAL_TIM_Encoder_MspDeInit(BspEncoderUser_HandleTable[timer].timer_handle);
    }

    return error;
}

Bsp_Error_t BspEncoder_Sample(const BspEncoderUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (timer >= BSP_ENCODER_USER_TIMER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else if (NULL == BspEncoderUser_HandleTable[timer].timer_handle)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        Bsp_Encoder_t *    handle          = &BspEncoderUser_HandleTable[timer];
        Bsp_Microsecond_t  previous_time   = handle->time;
        Bsp_EncoderPulse_t previous_pulses = handle->pulses;

        BspEncoder_SamplePulses(handle);

        double delta_time = (double)((double)(handle->time - previous_time) / BSP_TICK_MICROSECONDS_PER_SECOND);
        handle->raw_angular_rate = ((double)(handle->pulses - previous_pulses) * (handle->radians_per_pulse)) / delta_time;
        handle->angular_rate    += handle->smoothing_factor * (handle->raw_angular_rate - handle->angular_rate); /* Exponential moving average */
    }

    return error;
}

static void BspEncoder_SamplePulses(Bsp_Encoder_t *const handle)
{
    if ((NULL != handle) && (NULL != handle->timer_handle))
    {
        handle->time = BspTick_GetMicroseconds();

        handle->sampling                 = true; /* Flag to tell interrupt sampling in progress */
        handle->pulse_offset             = __HAL_TIM_GET_COUNTER(handle->timer_handle);
        handle->previous_periods_elapsed = handle->periods_elapsed;
        handle->sampling                 = false;

        handle->pulses  = (Bsp_EncoderPulse_t)((Bsp_EncoderPulse_t)handle->previous_periods_elapsed * handle->pulses_per_period);
        handle->pulses += (Bsp_EncoderPulse_t)handle->pulse_offset;
        handle->pulses *= BSP_ENCODER_PHASES;
    }
}

static void BspEncoder_TimerCallback(const Bsp_TimerHandle_t *const handle)
{
    BspEncoder_TimerCallbackHandler(BspEncoder_GetEncoderHandle(handle));
}

static inline void BspEncoder_TimerCallbackHandler(Bsp_Encoder_t *const handle)
{
    if ((NULL != handle) && (NULL != handle->timer_handle))
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(handle->timer_handle))
        {
            handle->periods_elapsed--;
        }
        else
        {
            handle->periods_elapsed++;
        }

        if (handle->sampling)
        {
            handle->pulse_offset             = __HAL_TIM_GET_COUNTER(handle->timer_handle);
            handle->previous_periods_elapsed = handle->periods_elapsed;
        }
    }
}

static Bsp_Encoder_t *BspEncoder_GetEncoderHandle(const Bsp_TimerHandle_t *const timer_handle)
{
    Bsp_Encoder_t *encoder_handle = NULL;

    for (BspEncoderUser_Timer_t user_timer = BSP_ENCODER_USER_TIMER_0; user_timer < BSP_ENCODER_USER_TIMER_MAX; user_timer++)
    {
        if (timer_handle == BspEncoderUser_HandleTable[user_timer].timer_handle)
        {
            encoder_handle = &BspEncoderUser_HandleTable[user_timer];
            break;
        }
    }

    return encoder_handle;
}