#include "rover_4ws.h"

#include "math.h"

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

#include "rover.h"
#include "rover_4ws_config.h"

#define ROVER_4WS_WHEEL_OFFSET (double)(3.14159265358979323846 / 2.0)

static Rover_Error_t Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle);
static Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle);
static inline Rover_Error_t Rover4ws_ErrorCheck(const Bsp_Error_t error_0,
                                                const Bsp_Error_t error_1,
                                                const Bsp_Error_t error_2,
                                                const Bsp_Error_t error_3);

Rover_Error_t Rover4ws_ConfigureSteering(const Rover4ws_Servo_t servo,
                                         const Bsp_Percent_t minimum_duty_cycle,
                                         const Bsp_Percent_t maximum_duty_cycle,
                                         const Bsp_Radian_t minimum_angle,
                                         const Bsp_Radian_t maximum_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (servo >= ROVER_4WS_SERVO_MAX)
    {
        error = ROVER_ERROR_PERIPHERAL;
    }
    else if (ROVER_MODE_CONFIGURE != Rover_GetMode())
    {
        error = ROVER_ERROR_MODE;
    }
    else
    {
        Rover4wsConfig_Servos[servo].minimum_duty_cycle = minimum_duty_cycle;
        Rover4wsConfig_Servos[servo].maximum_duty_cycle = maximum_duty_cycle;
        Rover4wsConfig_Servos[servo].minimum_angle      = minimum_angle;
        Rover4wsConfig_Servos[servo].maximum_angle      = maximum_angle;
    }

    return error;
}

Rover_Error_t Rover4ws_ConfigureMotor(const Rover4ws_Motor_t motor,
                                      const Bsp_Percent_t minimum_duty_cycle,
                                      const Bsp_Percent_t maximum_duty_cycle,
                                      const Bsp_RadiansPerSecond_t minimum_speed,
                                      const Bsp_RadiansPerSecond_t maximum_speed)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (motor >= ROVER_4WS_MOTOR_MAX)
    {
        error = ROVER_ERROR_PERIPHERAL;
    }
    else if (ROVER_MODE_CONFIGURE != Rover_GetMode())
    {
        error = ROVER_ERROR_MODE;
    }
    else
    {
        Rover4wsConfig_Motors[motor].minimum_duty_cycle = minimum_duty_cycle;
        Rover4wsConfig_Motors[motor].maximum_duty_cycle = maximum_duty_cycle;
        Rover4wsConfig_Motors[motor].minimum_speed      = minimum_speed;
        Rover4wsConfig_Motors[motor].maximum_speed      = maximum_speed;
    }

    return error;
}

Rover_Error_t Rover4ws_EnableSteering(void)
{
    return Rover4ws_ErrorCheck(BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_0]),
                               BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_2]),
                               BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_1]),
                               BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_3]));
}

Rover_Error_t Rover4ws_DisableSteering(void)
{
    return Rover4ws_ErrorCheck(BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_0]),
                               BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_2]),
                               BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_1]),
                               BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_3]));
}

Rover_Error_t Rover4ws_StartMotors(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

    return Rover4ws_ErrorCheck(BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0]),
                               BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2]),
                               BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1]),
                               BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3]));
}

Rover_Error_t Rover4ws_StopMotors(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

    return Rover4ws_ErrorCheck(BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0]),
                               BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2]),
                               BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1]),
                               BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3]));
}

Rover_Error_t Rover4ws_EnableEncoders(void)
{
    return Rover4ws_ErrorCheck(BspEncoder_Start(BSP_ENCODER_USER_TIMER_0),
                               BspEncoder_Start(BSP_ENCODER_USER_TIMER_1),
                               BspEncoder_Start(BSP_ENCODER_USER_TIMER_2),
                               BspEncoder_Start(BSP_ENCODER_USER_TIMER_3));
}

Rover_Error_t Rover4ws_SampleEncoders(void)
{
    return Rover4ws_ErrorCheck(BspEncoder_Sample(BSP_ENCODER_USER_TIMER_0),
                               BspEncoder_Sample(BSP_ENCODER_USER_TIMER_1),
                               BspEncoder_Sample(BSP_ENCODER_USER_TIMER_2),
                               BspEncoder_Sample(BSP_ENCODER_USER_TIMER_3));
}

Rover_Error_t Rover4ws_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    /* Check for divide by zero */
    if (0.0 != speed)
    {
        Rover_Radian_t steering_angle = atan((turn_rate * kRover4wsConfig_HalfWheelbase) / speed);
        error = Rover4ws_SetSteeringAngle(steering_angle);

        if (ROVER_ERROR_NONE == error)
        {
            error = Rover4ws_SetSpeed(speed, steering_angle);
        }
    }

    return error;
}

static Rover_Error_t Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    double radius              = kRover4wsConfig_HalfWheelbase / tan(steering_angle);
    double left_angular_speed  = (speed * (2 - (kRover4wsConfig_Tread / radius))) / kRover4wsConfig_DoubleWheelRadius;
    double right_angular_speed = (speed * (2 + (kRover4wsConfig_Tread / radius))) / kRover4wsConfig_DoubleWheelRadius;

    if (left_angular_speed < 0.0)
    {
        if ((BSP_ERROR_NONE != BspMotor_Reverse(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0])) ||
            (BSP_ERROR_NONE != BspMotor_Reverse(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2])))
        {
            error = ROVER_ERROR_BSP;
        }
    }
    else if ((BSP_ERROR_NONE != BspMotor_Forward(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0])) ||
             (BSP_ERROR_NONE != BspMotor_Forward(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2])))
    {
        error = ROVER_ERROR_BSP;
    }

    if (right_angular_speed < 0.0)
    {
        if ((BSP_ERROR_NONE != BspMotor_Reverse(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1])) ||
            (BSP_ERROR_NONE != BspMotor_Reverse(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3])))
        {
            error = ROVER_ERROR_BSP;
        }
    }
    else if ((BSP_ERROR_NONE != BspMotor_Forward(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1])) ||
             (BSP_ERROR_NONE != BspMotor_Forward(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3])))
    {
        error = ROVER_ERROR_BSP;
    }

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover4ws_ErrorCheck(BspMotor_SetSpeed(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0], fabs(left_angular_speed)),
                                    BspMotor_SetSpeed(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2], fabs(left_angular_speed)),
                                    BspMotor_SetSpeed(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1], fabs(right_angular_speed)),
                                    BspMotor_SetSpeed(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3], fabs(right_angular_speed)));
    }

    return error;
}

static Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle)
{
    double tan_delta        = tan(steering_angle);
    double scaled_wheelbase = kRover4wsConfig_HalfWheelbase * tan_delta;
    double offset           = kRover4wsConfig_HalfTread * tan_delta;

    double delta_left  = atan(scaled_wheelbase / (kRover4wsConfig_HalfWheelbase - offset));
    double delta_right = atan(scaled_wheelbase / (kRover4wsConfig_HalfWheelbase + offset));

    return Rover4ws_ErrorCheck(BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_0], (ROVER_4WS_WHEEL_OFFSET - delta_left)),
                               BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_1], (ROVER_4WS_WHEEL_OFFSET - delta_right)),
                               BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_2], (ROVER_4WS_WHEEL_OFFSET + delta_left)),
                               BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_3], (ROVER_4WS_WHEEL_OFFSET + delta_right)));
}

static inline Rover_Error_t Rover4ws_ErrorCheck(const Bsp_Error_t error_0,
                                                const Bsp_Error_t error_1,
                                                const Bsp_Error_t error_2,
                                                const Bsp_Error_t error_3)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    Rover_Error_t rover_error_0 = Rover_BspToRoverError(error_0);
    Rover_Error_t rover_error_1 = Rover_BspToRoverError(error_1);
    Rover_Error_t rover_error_2 = Rover_BspToRoverError(error_2);
    Rover_Error_t rover_error_3 = Rover_BspToRoverError(error_3);

    if (ROVER_ERROR_NONE != rover_error_0)
    {
        error = rover_error_0;
    }
    else if (ROVER_ERROR_NONE != rover_error_1)
    {
        error = rover_error_1;
    }
    else if (ROVER_ERROR_NONE != rover_error_2)
    {
        error = rover_error_2;
    }
    else if (ROVER_ERROR_NONE != rover_error_3)
    {
        error = rover_error_3;
    }

    return error;
}