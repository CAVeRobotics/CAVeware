#include "bsp_timer_user.h"

#include <stdbool.h>

#include "tim.h"

Bsp_Timer_t BspTimerUser_HandleTable[BSP_TIMER_USER_TIMER_MAX] = {
    [BSP_TIMER_USER_TIMER_0] = {
        .timer_handle          = &htim7,
        .has_interrupt         = true,
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