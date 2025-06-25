#include "bsp_tick_user.h"

#include "tim.h"

#include "bsp.h"

Bsp_TimerHandle_t *const BspTickUser_TimerHandle = &htim6;