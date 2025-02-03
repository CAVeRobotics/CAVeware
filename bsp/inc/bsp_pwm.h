#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "bsp_pwm_user.h"
#include "bsp_types.h"

BspTypes_Error_t BspPwm_Start(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel);
BspTypes_Error_t BspPwm_Stop(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel);
BspTypes_Error_t BspPwm_SetPeriod(const BspPwmUser_Timer_t timer, const BspTypes_Microsecond_t period);
BspTypes_Error_t BspPwm_SetDutyCycle(const BspPwmUser_Timer_t timer, const BspTypes_TimerChannel_t channel, const BspTypes_Percent_t duty_cycle);

#endif /* BSP_PWM_H */