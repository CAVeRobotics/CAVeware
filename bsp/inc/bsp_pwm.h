#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp.h"
#include "bsp_pwm_user.h"

Bsp_Error_t BspPwm_Start(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel);
Bsp_Error_t BspPwm_Stop(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel);
Bsp_Error_t BspPwm_SetPeriod(const BspPwmUser_Timer_t timer, const Bsp_Microsecond_t period);
Bsp_Error_t BspPwm_SetDutyCycle(const BspPwmUser_Timer_t timer, const Bsp_TimerChannel_t channel, const Bsp_Percent_t duty_cycle);

#endif /* BSP_PWM_H */