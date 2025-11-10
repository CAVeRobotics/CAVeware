#include "rover_4ws.h"

#include <math.h>

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_tick.h"

#include "accelerometer.h"
#include "gyroscope.h"

#include "cavebot.h"
#include "cavebot_pid.h"
#include "cavebot_user.h"

#define ROVER_4WS_WHEEL_OFFSET (double)(3.14159265358979323846 / 2.0)

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4ws_Tread       = 0.493800;
static const Bsp_Meter_t kRover4ws_Wheelbase   = 0.466028;
static const Bsp_Meter_t kRover4ws_WheelRadius = 0.080000;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4ws_HalfTread         = kRover4ws_Tread / 2;
static const Bsp_Meter_t kRover4ws_HalfWheelbase     = kRover4ws_Wheelbase / 2;
static const Bsp_Meter_t kRover4ws_DoubleWheelRadius = kRover4ws_WheelRadius * 2;

/* TODO CVW-21 read from config */
static CavebotPid_Handle_t Rover4ws_SteeringPid = {
    0
};

static Bsp_MetersPerSecond_t Rover4ws_CommandedSpeed = 0.0;

static Cavebot_Error_t Rover4ws_EnableSteering(void);
static Cavebot_Error_t Rover4ws_DisableSteering(void);
static Cavebot_Error_t Rover4ws_StartMotors(void);
static Cavebot_Error_t Rover4ws_StopMotors(void);
static void Rover4ws_SetSpeed(const Bsp_MetersPerSecond_t speed, const Bsp_Radian_t steering_angle);
static Cavebot_Error_t Rover4ws_MotorSpeedControl(const CavebotUser_Motor_t motor);
static Cavebot_Error_t Rover4ws_SetMotorSpeed(const CavebotUser_Motor_t motor, const Bsp_RadiansPerSecond_t speed);
static Cavebot_Error_t Rover4ws_SetSteeringAngle(const Bsp_Radian_t steering_angle);
static Cavebot_Error_t Rover4ws_BspErrorCheck(const Bsp_Error_t error_0,
                                              const Bsp_Error_t error_1,
                                              const Bsp_Error_t error_2,
                                              const Bsp_Error_t error_3);

Cavebot_Error_t Rover4ws_ConfigureSteering(const CavebotUser_Servo_t servo,
                                           const Bsp_Percent_t minimum_duty_cycle,
                                           const Bsp_Percent_t maximum_duty_cycle,
                                           const Bsp_Radian_t minimum_angle,
                                           const Bsp_Radian_t maximum_angle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (servo >= CAVEBOT_USER_SERVO_MAX)
    {
        error = CAVEBOT_ERROR_PERIPHERAL;
    }
    else if (Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else
    {
        CavebotUser_Servos[servo].minimum_duty_cycle = minimum_duty_cycle;
        CavebotUser_Servos[servo].maximum_duty_cycle = maximum_duty_cycle;
        CavebotUser_Servos[servo].minimum_angle      = minimum_angle;
        CavebotUser_Servos[servo].maximum_angle      = maximum_angle;
    }

    return error;
}

Cavebot_Error_t Rover4ws_ConfigureMotor(const CavebotUser_Motor_t motor,
                                        const Bsp_Percent_t minimum_duty_cycle,
                                        const Bsp_Percent_t maximum_duty_cycle,
                                        const Bsp_RadiansPerSecond_t minimum_speed,
                                        const Bsp_RadiansPerSecond_t maximum_speed)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (motor >= CAVEBOT_USER_MOTOR_MAX)
    {
        error = CAVEBOT_ERROR_PERIPHERAL;
    }
    else if (Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else
    {
        CavebotUser_Motors[motor].minimum_duty_cycle = minimum_duty_cycle;
        CavebotUser_Motors[motor].maximum_duty_cycle = maximum_duty_cycle;
        CavebotUser_Motors[motor].minimum_speed      = minimum_speed;
        CavebotUser_Motors[motor].maximum_speed      = maximum_speed;
    }

    return error;
}

Cavebot_Error_t Rover4ws_ConfigureMotorPid(const CavebotUser_Motor_t motor, const double kp, const double ki, const double kd)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (motor >= CAVEBOT_USER_MOTOR_MAX)
    {
        error = CAVEBOT_ERROR_PERIPHERAL;
    }
    else if (Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else
    {
        CavebotUser_MotorsPid[motor].kp = kp;
        CavebotUser_MotorsPid[motor].ki = ki;
        CavebotUser_MotorsPid[motor].kd = kd;
    }

    return error;
}

Cavebot_Error_t Rover4ws_ConfigureSteeringPid(const double kp, const double ki, const double kd)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else
    {
        Rover4ws_SteeringPid.kp = kp;
        Rover4ws_SteeringPid.ki = ki;
        Rover4ws_SteeringPid.kd = kd;
    }

    return error;
}

Cavebot_Error_t Rover4ws_ConfigureEncoder(const CavebotUser_Motor_t motor, const double smoothing_factor)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_PERIPHERAL;

    if (motor >= CAVEBOT_USER_MOTOR_MAX)
    {
    }
    else if (Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else if (CAVEBOT_ERROR_NONE == error)
    {
        BspEncoderUser_Timer_t encoder = CavebotUser_Encoders[motor];

        error = Cavebot_BspToCavebotError(BspEncoder_Stop(encoder));

        if (CAVEBOT_ERROR_NONE == error)
        {
            BspEncoderUser_HandleTable[encoder].smoothing_factor = smoothing_factor;

            error = Cavebot_BspToCavebotError(BspEncoder_Start(encoder));
        }
    }

    return error;
}

Cavebot_Error_t Rover4ws_EnableEncoders(void)
{
    return Rover4ws_BspErrorCheck(BspEncoder_Start(BSP_ENCODER_USER_TIMER_0),
                                  BspEncoder_Start(BSP_ENCODER_USER_TIMER_1),
                                  BspEncoder_Start(BSP_ENCODER_USER_TIMER_2),
                                  BspEncoder_Start(BSP_ENCODER_USER_TIMER_3));
}

Cavebot_Error_t Rover4ws_DisableEncoders(void)
{
    return Rover4ws_BspErrorCheck(BspEncoder_Stop(BSP_ENCODER_USER_TIMER_0),
                                  BspEncoder_Stop(BSP_ENCODER_USER_TIMER_1),
                                  BspEncoder_Stop(BSP_ENCODER_USER_TIMER_2),
                                  BspEncoder_Stop(BSP_ENCODER_USER_TIMER_3));
}

Cavebot_Error_t Rover4ws_SampleEncoders(void)
{
    return Rover4ws_BspErrorCheck(BspEncoder_Sample(BSP_ENCODER_USER_TIMER_0),
                                  BspEncoder_Sample(BSP_ENCODER_USER_TIMER_1),
                                  BspEncoder_Sample(BSP_ENCODER_USER_TIMER_2),
                                  BspEncoder_Sample(BSP_ENCODER_USER_TIMER_3));
}

Cavebot_Error_t Rover4ws_EnableSpeedControl(void)
{
    return Rover4ws_ErrorCheck(CavebotPid_Enable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_0]),
                               CavebotPid_Enable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_1]),
                               CavebotPid_Enable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_2]),
                               CavebotPid_Enable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_3]));
}

Cavebot_Error_t Rover4ws_DisableSpeedControl(void)
{
    return Rover4ws_ErrorCheck(CavebotPid_Disable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_0]),
                               CavebotPid_Disable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_1]),
                               CavebotPid_Disable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_2]),
                               CavebotPid_Disable(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_3]));
}

Cavebot_Error_t Rover4ws_EnableSteeringControl(void)
{
    return CavebotPid_Enable(&Rover4ws_SteeringPid);
}

Cavebot_Error_t Rover4ws_DisableSteeringControl(void)
{
    return CavebotPid_Disable(&Rover4ws_SteeringPid);
}

Cavebot_Error_t Rover4ws_Arm(void)
{
    Cavebot_Error_t error = Rover4ws_StartMotors();

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_EnableSteering();
    }

    return error;
}

Cavebot_Error_t Rover4ws_Disarm(void)
{
    Cavebot_Error_t error = Rover4ws_StopMotors();

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_DisableSteering();
    }

    return error;
}

Cavebot_Error_t Rover4ws_Task(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (Cavebot_IsArmed())
    {
        Gyroscope_Reading_t reading = {
            .x = 0.0,
            .y = 0.0,
            .z = 0.0
        };

        (void)Rover4ws_SampleEncoders();
        (void)Gyroscope_Read(&CavebotUser_Gyroscope, &reading);

        /* TODO SD-126 test with steering control coupled and decoupled from wheel speed control */
        (void)CavebotPid_Update(&Rover4ws_SteeringPid, reading.z, BspTick_GetMicroseconds());
        error = Rover4ws_SetSteeringAngle(Rover4ws_SteeringPid.output);

        if (CAVEBOT_ERROR_NONE == error)
        {
            Rover4ws_SetSpeed(Rover4ws_CommandedSpeed, Rover4ws_SteeringPid.output);
            error = Rover4ws_ErrorCheck(Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_0),
                                        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_1),
                                        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_2),
                                        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_3));
        }
    }

    return error;
}

Cavebot_Error_t Rover4ws_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (!Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    /* Check for divide by zero */
    else if (0.0 != speed)
    {
        Bsp_Radian_t steering_angle = atan((turn_rate * kRover4ws_HalfWheelbase) / speed);

        Rover4ws_CommandedSpeed      = speed;
        Rover4ws_SteeringPid.command = steering_angle;
    }

    return error;
}

Cavebot_Error_t Rover4ws_ErrorCheck(const Cavebot_Error_t error_0,
                                    const Cavebot_Error_t error_1,
                                    const Cavebot_Error_t error_2,
                                    const Cavebot_Error_t error_3)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (CAVEBOT_ERROR_NONE != error_0)
    {
        error = error_0;
    }
    else if (CAVEBOT_ERROR_NONE != error_1)
    {
        error = error_1;
    }
    else if (CAVEBOT_ERROR_NONE != error_2)
    {
        error = error_2;
    }
    else if (CAVEBOT_ERROR_NONE != error_3)
    {
        error = error_3;
    }

    return error;
}

static Cavebot_Error_t Rover4ws_EnableSteering(void)
{
    Cavebot_Error_t error = CavebotPid_Reset(&Rover4ws_SteeringPid);

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_0]),
                                       BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_2]),
                                       BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_1]),
                                       BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_3]));
    }

    return error;
}

static Cavebot_Error_t Rover4ws_DisableSteering(void)
{
    return Rover4ws_BspErrorCheck(BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_0]),
                                  BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_2]),
                                  BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_1]),
                                  BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_3]));
}

static Cavebot_Error_t Rover4ws_StartMotors(void)
{
    Cavebot_Error_t error = Rover4ws_ErrorCheck(CavebotPid_Reset(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_0]),
                                                CavebotPid_Reset(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_2]),
                                                CavebotPid_Reset(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_1]),
                                                CavebotPid_Reset(&CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_3]));

    Rover4ws_CommandedSpeed = 0.0;

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Cavebot_BspToCavebotError(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_SLEEP, BSP_GPIO_STATE_SET));
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    return error;
}

static Cavebot_Error_t Rover4ws_StopMotors(void)
{
    Cavebot_Error_t error = Cavebot_BspToCavebotError(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_SLEEP, BSP_GPIO_STATE_RESET));

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    return error;
}

static void Rover4ws_SetSpeed(const Bsp_MetersPerSecond_t speed, const Bsp_Radian_t steering_angle)
{
    double radius              = kRover4ws_HalfWheelbase / tan(steering_angle);
    double left_angular_speed  = (speed * (2 - (kRover4ws_Tread / radius))) / kRover4ws_DoubleWheelRadius;
    double right_angular_speed = (speed * (2 + (kRover4ws_Tread / radius))) / kRover4ws_DoubleWheelRadius;

    CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_0].command = left_angular_speed;
    CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_2].command = left_angular_speed;
    CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_1].command = right_angular_speed;
    CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_3].command = right_angular_speed;
}

static Cavebot_Error_t Rover4ws_MotorSpeedControl(const CavebotUser_Motor_t motor)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_PERIPHERAL;

    if (motor < CAVEBOT_USER_MOTOR_MAX)
    {
        error = CavebotPid_Update(&CavebotUser_MotorsPid[motor], BspEncoderUser_HandleTable[CavebotUser_Encoders[motor]].angular_rate, BspTick_GetMicroseconds());

        if (CAVEBOT_ERROR_NONE == error)
        {
            error = Rover4ws_SetMotorSpeed(motor, CavebotUser_MotorsPid[motor].output);
        }
    }

    return error;
}

static Cavebot_Error_t Rover4ws_SetMotorSpeed(const CavebotUser_Motor_t motor, const Bsp_RadiansPerSecond_t speed)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_PERIPHERAL;

    if (motor < CAVEBOT_USER_MOTOR_MAX)
    {
        if (speed < 0.0)
        {
            error = Cavebot_BspToCavebotError(BspMotor_Reverse(&CavebotUser_Motors[motor]));
        }
        else
        {
            error = Cavebot_BspToCavebotError(BspMotor_Forward(&CavebotUser_Motors[motor]));
        }

        if (CAVEBOT_ERROR_NONE == error)
        {
            error = Cavebot_BspToCavebotError(BspMotor_SetSpeed(&CavebotUser_Motors[motor], fabs(speed)));
        }
    }

    return error;
}

static Cavebot_Error_t Rover4ws_SetSteeringAngle(const Bsp_Radian_t steering_angle)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_VALUE;

    double      tan_delta        = tan(steering_angle);
    Bsp_Meter_t scaled_wheelbase = kRover4ws_HalfWheelbase * tan_delta;
    Bsp_Meter_t offset           = kRover4ws_HalfTread * tan_delta;

    Bsp_Radian_t delta_left  = atan(scaled_wheelbase / (kRover4ws_HalfWheelbase - offset));
    Bsp_Radian_t delta_right = atan(scaled_wheelbase / (kRover4ws_HalfWheelbase + offset));

    if (Bsp_CompareDoubleSigns(&delta_left, &delta_right))
    {
        error = Rover4ws_BspErrorCheck(BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_0], (ROVER_4WS_WHEEL_OFFSET - delta_left)),
                                       BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_1], (ROVER_4WS_WHEEL_OFFSET - delta_right)),
                                       BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_2], (ROVER_4WS_WHEEL_OFFSET + delta_left)),
                                       BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_3], (ROVER_4WS_WHEEL_OFFSET + delta_right)));
    }

    return error;
}

static Cavebot_Error_t Rover4ws_BspErrorCheck(const Bsp_Error_t error_0,
                                              const Bsp_Error_t error_1,
                                              const Bsp_Error_t error_2,
                                              const Bsp_Error_t error_3)
{
    Cavebot_Error_t rover_error_0 = Cavebot_BspToCavebotError(error_0);
    Cavebot_Error_t rover_error_1 = Cavebot_BspToCavebotError(error_1);
    Cavebot_Error_t rover_error_2 = Cavebot_BspToCavebotError(error_2);
    Cavebot_Error_t rover_error_3 = Cavebot_BspToCavebotError(error_3);

    return Rover4ws_ErrorCheck(rover_error_0, rover_error_1, rover_error_2, rover_error_3);
}