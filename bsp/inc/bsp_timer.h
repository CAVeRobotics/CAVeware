#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>

#include "bsp.h"
#include "bsp_timer_user.h"

Bsp_Error_t BspTimer_Start(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_Stop(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_Reset(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_Sample(const BspTimerUser_Timer_t timer);
Bsp_Error_t BspTimer_SetPeriodElapsedCallback(const BspTimerUser_Timer_t timer, void (*period_elapsed_callback)(const Bsp_Timer_t *const timer));

#endif /* BSP_TIMER_H */