#ifndef BSP_MOTOR_H
#define BPS_MOTOR_H

#include "bsp.h"
#include "bsp_pwm_user.h"

typedef enum
{
    BSP_MOTOR_DIRECTION_FORWARD,
    BSP_MOTOR_DIRECTION_REVERSE
} BspMotor_Direction_t;

typedef struct
{
    BspPwmUser_Timer_t timer;
    Bsp_TimerChannel_t channel;
} BspMotor_Phase_t;

typedef struct
{
    BspMotor_Phase_t forward_phase;
    BspMotor_Phase_t reverse_phase;
    Bsp_Percent_t minimum_duty_cycle;
    Bsp_Percent_t maximum_duty_cycle;
    Bsp_RadiansPerSecond_t minimum_speed;
    Bsp_RadiansPerSecond_t maximum_speed;
    BspMotor_Direction_t direction;
} BspMotor_Handle_t;

Bsp_Error_t BspMotor_Start(BspMotor_Handle_t *const handle);
Bsp_Error_t BspMotor_Stop(BspMotor_Handle_t *const handle);
Bsp_Error_t BspMotor_Forward(BspMotor_Handle_t *const handle);
Bsp_Error_t BspMotor_Reverse(BspMotor_Handle_t *const handle);
Bsp_Error_t BspMotor_SetDirection(BspMotor_Handle_t *const handle, const BspMotor_Direction_t direction);
Bsp_Error_t BspMotor_SetDutyCycle(BspMotor_Handle_t *const handle, const Bsp_Percent_t duty_cycle);
Bsp_Error_t BspMotor_SetSpeed(BspMotor_Handle_t *const handle, const Bsp_RadiansPerSecond_t speed);
Bsp_Error_t BspMotor_Brake(BspMotor_Handle_t *const handle);
Bsp_Error_t BspMotor_Coast(BspMotor_Handle_t *const handle);

#endif /* BSP_MOTOR_H */