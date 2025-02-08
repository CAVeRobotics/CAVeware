#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include "bsp_pwm_user.h"
#include "bsp.h"

typedef struct
{
    BspPwmUser_Timer_t timer;
    Bsp_TimerChannel_t channel;
    Bsp_Percent_t minimum_duty_cycle;
    Bsp_Percent_t maximum_duty_cycle;
    double minimum_angle;
    double maximum_angle;
} BspServo_Handle_t;

Bsp_Error_t BspServo_Start(BspServo_Handle_t *const handle);
Bsp_Error_t BspServo_SetDutyCycle(BspServo_Handle_t *const handle, const Bsp_Percent_t duty_cycle);
Bsp_Error_t BspServo_SetAngle(BspServo_Handle_t *const handle, const double angle);

#endif /* BSP_SERVO_H */