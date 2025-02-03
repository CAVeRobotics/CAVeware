#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include "bsp_pwm_user.h"
#include "bsp_types.h"

typedef struct {
    BspPwmUser_Timer_t timer;
    BspTypes_TimerChannel_t channel;
    BspTypes_Percent_t minimum_duty_cycle;
    BspTypes_Percent_t maximum_duty_cycle;
    double minimum_angle;
    double maximum_angle;
} BspServo_Handle_t;

BspTypes_Error_t BspServo_Start(BspServo_Handle_t *const handle);
BspTypes_Error_t BspServo_SetDutyCycle(BspServo_Handle_t *const handle, const BspTypes_Percent_t duty_cycle);
BspTypes_Error_t BspServo_SetAngle(BspServo_Handle_t *const handle, const double angle);

#endif /* BSP_SERVO_H */