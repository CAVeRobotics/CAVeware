#include "bsp_timer_user.h"

#include <stdbool.h>

#include "tim.h"

Bsp_Timer_t BspTimerUser_HandleTable[BSP_TIMER_USER_TIMER_MAX] = {
    [BSP_TIMER_USER_TIMER_0] = {
        .timer_handle          = &htim7,
        .counts_elapsed        = 0U,
        .counts_elapsed_shadow = 0U,
        .counts_offset         = 0U,
        .sampling              = false,
        .period_elapsed        = {
            .function = NULL,
            .arg      = NULL,
        },
        .counts = 0U,
    },
};

Bsp_Timer_t *BspTimerUser_GetTimer(const Bsp_TimerHandle_t *const timer_handle)
{
    Bsp_Timer_t *timer = NULL;

    if (timer_handle == BspTimerUser_HandleTable[BSP_TIMER_USER_TIMER_0].timer_handle)
    {
        timer = &BspTimerUser_HandleTable[BSP_TIMER_USER_TIMER_0];
    }

    return timer;
}