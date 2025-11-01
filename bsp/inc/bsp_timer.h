#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>

#include "bsp.h"
#include "bsp_timer_user.h"

Bsp_Error_t BspTimer_Start(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_Stop(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_Reset(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_Sample(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_RegisterPeriodElapsedCallback(const BspTimerUser_Timer_t timer, const Bsp_Callback_t *const callback);

#endif /* BSP_TIMER_H */