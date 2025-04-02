#include "rover_4ws.h"

#include "math.h"

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

#include "rover.h"
#include "rover_4ws_config.h"

#define ROVER_4WS_WHEEL_OFFSET (double)(3.14159265358979323846 / 2.0)

static Rover_Error_t Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle);
static Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle);
static Rover_Error_t Rover4ws_BspErrorCheck(const Bsp_Error_t error_0,
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
    else if (Rover_IsArmed())
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
    else if (Rover_IsArmed())
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

Rover_Error_t Rover4ws_ConfigureEncoder(const Rover4ws_Motor_t motor, const double smoothing_factor)
{
    Rover_Error_t          error   = ROVER_ERROR_NONE;
    BspEncoderUser_Timer_t encoder = BSP_ENCODER_USER_TIMER_MAX;

    switch (motor)
    {
    case ROVER_4WS_MOTOR_0:
        encoder = BSP_ENCODER_USER_TIMER_0;
        break;
    case ROVER_4WS_MOTOR_1:
        encoder = BSP_ENCODER_USER_TIMER_1;
        break;
    case ROVER_4WS_MOTOR_2:
        encoder = BSP_ENCODER_USER_TIMER_2;
        break;
    case ROVER_4WS_MOTOR_3:
        encoder = BSP_ENCODER_USER_TIMER_3;
        break;
    default:
        error = ROVER_ERROR_PERIPHERAL;
        break;
    }

    if (Rover_IsArmed())
    {
        error = ROVER_ERROR_MODE;
    }
    else if (ROVER_ERROR_NONE == error)
    {
        error = Rover_BspToRoverError(BspEncoder_Stop(encoder));

        if (ROVER_ERROR_NONE == error)
        {
            BspEncoderUser_HandleTable[encoder].smoothing_factor = smoothing_factor;

            error = Rover_BspToRoverError(BspEncoder_Start(encoder));
        }
    }

    return error;
}

Rover_Error_t Rover4ws_EnableSteering(void)
{
    return Rover4ws_BspErrorCheck(BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_0]),
                                  BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_2]),
                                  BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_1]),
                                  BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_3]));
}

Rover_Error_t Rover4ws_DisableSteering(void)
{
    return Rover4ws_BspErrorCheck(BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_0]),
                                  BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_2]),
                                  BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_1]),
                                  BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_3]));
}

Rover_Error_t Rover4ws_StartMotors(void)
{
    BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_SLEEP, BSP_GPIO_STATE_SET);

    return Rover4ws_BspErrorCheck(BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0]),
                                  BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2]),
                                  BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1]),
                                  BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3]));
}

Rover_Error_t Rover4ws_StopMotors(void)
{
    BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_SLEEP, BSP_GPIO_STATE_RESET);

    return Rover4ws_BspErrorCheck(BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0]),
                                  BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_2]),
                                  BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_1]),
                                  BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_3]));
}

Rover_Error_t Rover4ws_EnableEncoders(void)
{
    return Rover4ws_BspErrorCheck(BspEncoder_Start(BSP_ENCODER_USER_TIMER_0),
                                  BspEncoder_Start(BSP_ENCODER_USER_TIMER_1),
                                  BspEncoder_Start(BSP_ENCODER_USER_TIMER_2),
                                  BspEncoder_Start(BSP_ENCODER_USER_TIMER_3));
}

Rover_Error_t Rover4ws_DisableEncoders(void)
{
    return Rover4ws_BspErrorCheck(BspEncoder_Stop(BSP_ENCODER_USER_TIMER_0),
                                  BspEncoder_Stop(BSP_ENCODER_USER_TIMER_1),
                                  BspEncoder_Stop(BSP_ENCODER_USER_TIMER_2),
                                  BspEncoder_Stop(BSP_ENCODER_USER_TIMER_3));
}

Rover_Error_t Rover4ws_SampleEncoders(void)
{
    return Rover4ws_BspErrorCheck(BspEncoder_Sample(BSP_ENCODER_USER_TIMER_0),
                                  BspEncoder_Sample(BSP_ENCODER_USER_TIMER_1),
                                  BspEncoder_Sample(BSP_ENCODER_USER_TIMER_2),
                                  BspEncoder_Sample(BSP_ENCODER_USER_TIMER_3));
}

Rover_Error_t Rover4ws_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (!Rover_IsArmed())
    {
        error = ROVER_ERROR_MODE;
    }
    /* Check for divide by zero */
    else if (0.0 != speed)
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

Rover_Error_t Rover4ws_ErrorCheck(const Rover_Error_t error_0,
                                  const Rover_Error_t error_1,
                                  const Rover_Error_t error_2,
                                  const Rover_Error_t error_3)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (ROVER_ERROR_NONE != error_0)
    {
        error = error_0;
    }
    else if (ROVER_ERROR_NONE != error_1)
    {
        error = error_1;
    }
    else if (ROVER_ERROR_NONE != error_2)
    {
        error = error_2;
    }
    else if (ROVER_ERROR_NONE != error_3)
    {
        error = error_3;
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
        error = Rover4ws_BspErrorCheck(BspMotor_SetSpeed(&Rover4wsConfig_Motors[ROVER_4WS_MOTOR_0], fabs(left_angular_speed)),
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

    return Rover4ws_BspErrorCheck(BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_0], (ROVER_4WS_WHEEL_OFFSET - delta_left)),
                                  BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_1], (ROVER_4WS_WHEEL_OFFSET - delta_right)),
                                  BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_2], (ROVER_4WS_WHEEL_OFFSET + delta_left)),
                                  BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_SERVO_3], (ROVER_4WS_WHEEL_OFFSET + delta_right)));
}

static Rover_Error_t Rover4ws_BspErrorCheck(const Bsp_Error_t error_0,
                                            const Bsp_Error_t error_1,
                                            const Bsp_Error_t error_2,
                                            const Bsp_Error_t error_3)
{
    Rover_Error_t rover_error_0 = Rover_BspToRoverError(error_0);
    Rover_Error_t rover_error_1 = Rover_BspToRoverError(error_1);
    Rover_Error_t rover_error_2 = Rover_BspToRoverError(error_2);
    Rover_Error_t rover_error_3 = Rover_BspToRoverError(error_3);

    return Rover4ws_ErrorCheck(rover_error_0, rover_error_1, rover_error_2, rover_error_3);
}