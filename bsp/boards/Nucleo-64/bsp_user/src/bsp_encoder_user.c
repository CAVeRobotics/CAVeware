#include "bsp_encoder_user.h"

#include "tim.h"

#include "bsp.h"

Bsp_Encoder_t BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_MAX] = {
    [BSP_ENCODER_USER_TIMER_0] = {
        .timer_handle             = &htim3,
        .pulses_per_period        = 0,
        .smoothing_factor         = 0.5,
        .mode                     = BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON,
        .pulses_per_rotation      = 753.2,
        .sampling                 = false,
        .pulse_offset             = 0U,
        .previous_periods_elapsed = 0,
        .periods_elapsed          = 0,
        .time                     = 0U,
        .pulses                   = 0,
        .raw_angular_rate         = 0,
        .angular_rate             = 0,
    },
    [BSP_ENCODER_USER_TIMER_1] = {
        .timer_handle             = &htim5,
        .pulses_per_period        = 0,
        .smoothing_factor         = 0.5,
        .mode                     = BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON,
        .pulses_per_rotation      = 753.2,
        .sampling                 = false,
        .pulse_offset             = 0U,
        .previous_periods_elapsed = 0,
        .periods_elapsed          = 0,
        .time                     = 0U,
        .pulses                   = 0,
        .raw_angular_rate         = 0,
        .angular_rate             = 0,
    },
};

Bsp_Encoder_t *BspEncoderUser_GetEncoderHandle(const Bsp_TimerHandle_t *const timer_handle)
{
    Bsp_Encoder_t *encoder_handle = NULL;

    if (timer_handle == BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].timer_handle)
    {
        encoder_handle = &BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0];
    }
    else if (timer_handle == BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].timer_handle)
    {
        encoder_handle = &BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1];
    }

    return encoder_handle;
}