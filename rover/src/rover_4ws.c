#include "rover_4ws.h"

#include "math.h"

#include "bsp.h"
#include "bsp_encoder.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_tick.h"

#include "rover.h"
#include "rover_4ws_config.h"
#include "rover_imu.h"
#include "rover_pid.h"

#define ROVER_4WS_WHEEL_OFFSET     (double)(3.14159265358979323846 / 2.0)
#define ROVER_4WS_DOUBLE_SIGN_MASK 0x8000000000000000U

static Rover_MetersPerSecond_t Rover4ws_CommandedSpeed = 0.0;

static void Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle);
static Rover_Error_t Rover4ws_MotorSpeedControl(const Rover4wsConfig_Motor_t motor);
static Rover_Error_t Rover4ws_SetMotorSpeed(const Rover4wsConfig_Motor_t motor, const Rover_RadiansPerSecond_t speed);
static Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle);
static Rover_Error_t Rover4ws_BspErrorCheck(const Bsp_Error_t error_0,
                                            const Bsp_Error_t error_1,
                                            const Bsp_Error_t error_2,
                                            const Bsp_Error_t error_3);
static inline bool Rover4ws_CompareDoubleSigns(const double value_1, const double value_2);

Rover_Error_t Rover4ws_ConfigureSteering(const Rover4wsConfig_Servo_t servo,
                                         const Bsp_Percent_t minimum_duty_cycle,
                                         const Bsp_Percent_t maximum_duty_cycle,
                                         const Bsp_Radian_t minimum_angle,
                                         const Bsp_Radian_t maximum_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (servo >= ROVER_4WS_CONFIG_SERVO_MAX)
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

Rover_Error_t Rover4ws_ConfigureMotor(const Rover4wsConfig_Motor_t motor,
                                      const Bsp_Percent_t minimum_duty_cycle,
                                      const Bsp_Percent_t maximum_duty_cycle,
                                      const Bsp_RadiansPerSecond_t minimum_speed,
                                      const Bsp_RadiansPerSecond_t maximum_speed)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (motor >= ROVER_4WS_CONFIG_MOTOR_MAX)
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

Rover_Error_t Rover4ws_ConfigureMotorPid(const Rover4wsConfig_Motor_t motor, const double kp, const double ki, const double kd)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (motor >= ROVER_4WS_CONFIG_MOTOR_MAX)
    {
        error = ROVER_ERROR_PERIPHERAL;
    }
    else if (Rover_IsArmed())
    {
        error = ROVER_ERROR_MODE;
    }
    else
    {
        Rover4wsConfig_MotorsPid[motor].kp = kp;
        Rover4wsConfig_MotorsPid[motor].ki = ki;
        Rover4wsConfig_MotorsPid[motor].kd = kd;
    }

    return error;
}

Rover_Error_t Rover4ws_ConfigureSteeringPid(const double kp, const double ki, const double kd)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (Rover_IsArmed())
    {
        error = ROVER_ERROR_MODE;
    }
    else
    {
        Rover4wsConfig_SteeringPid.kp = kp;
        Rover4wsConfig_SteeringPid.ki = ki;
        Rover4wsConfig_SteeringPid.kd = kd;
    }

    return error;
}


Rover_Error_t Rover4ws_ConfigureEncoder(const Rover4wsConfig_Motor_t motor, const double smoothing_factor)
{
    Rover_Error_t error = ROVER_ERROR_PERIPHERAL;

    if (motor >= ROVER_4WS_CONFIG_MOTOR_MAX)
    {
    }
    else if (Rover_IsArmed())
    {
        error = ROVER_ERROR_MODE;
    }
    else if (ROVER_ERROR_NONE == error)
    {
        BspEncoderUser_Timer_t encoder = Rover4wsConfig_Encoders[motor];

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
    Rover_Error_t error = RoverPid_Reset(&Rover4wsConfig_SteeringPid);

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_0]),
                                       BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_2]),
                                       BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_1]),
                                       BspServo_Start(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_3]));
    }

    return error;
}

Rover_Error_t Rover4ws_DisableSteering(void)
{
    return Rover4ws_BspErrorCheck(BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_0]),
                                  BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_2]),
                                  BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_1]),
                                  BspServo_Stop(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_3]));
}

Rover_Error_t Rover4ws_StartMotors(void)
{
    Rover_Error_t error = Rover4ws_ErrorCheck(RoverPid_Reset(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_0]),
                                              RoverPid_Reset(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_2]),
                                              RoverPid_Reset(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_1]),
                                              RoverPid_Reset(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_3]));

    Rover4ws_CommandedSpeed = 0.0;

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover_BspToRoverError(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_SLEEP, BSP_GPIO_STATE_SET));
    }

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_0]),
                                       BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_2]),
                                       BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_1]),
                                       BspMotor_Start(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_3]));
    }

    return error;
}

Rover_Error_t Rover4ws_StopMotors(void)
{
    Rover_Error_t error = Rover_BspToRoverError(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_SLEEP, BSP_GPIO_STATE_RESET));

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_0]),
                                       BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_2]),
                                       BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_1]),
                                       BspMotor_Stop(&Rover4wsConfig_Motors[ROVER_4WS_CONFIG_MOTOR_3]));
    }

    return error;
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

Rover_Error_t Rover4ws_EnableSpeedControl(void)
{
    return Rover4ws_ErrorCheck(RoverPid_Enable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_0]),
                               RoverPid_Enable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_1]),
                               RoverPid_Enable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_2]),
                               RoverPid_Enable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_3]));
}

Rover_Error_t Rover4ws_DisableSpeedControl(void)
{
    return Rover4ws_ErrorCheck(RoverPid_Disable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_0]),
                               RoverPid_Disable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_1]),
                               RoverPid_Disable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_2]),
                               RoverPid_Disable(&Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_3]));
}

Rover_Error_t Rover4ws_EnableSteeringControl(void)
{
    return RoverPid_Enable(&Rover4wsConfig_SteeringPid);
}

Rover_Error_t Rover4ws_DisableSteeringControl(void)
{
    return RoverPid_Disable(&Rover4wsConfig_SteeringPid);
}

Rover_Error_t Rover4ws_Task(void)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (Rover_IsArmed())
    {
        Rover_GyroscopeReading_t gyroscope = {
            .x = 0.0,
            .y = 0.0,
            .z = 0.0
        };

        (void)Rover4ws_SampleEncoders();
        (void)RoverImu_ReadGyroscope(&gyroscope);

        /* TODO SD-126 test with steering control coupled and decoupled from wheel speed control */
        (void)RoverPid_Update(&Rover4wsConfig_SteeringPid, gyroscope.z, BspTick_GetMicroseconds());
        error = Rover4ws_SetSteeringAngle(Rover4wsConfig_SteeringPid.output);

        if (ROVER_ERROR_NONE == error)
        {
            Rover4ws_SetSpeed(Rover4ws_CommandedSpeed, Rover4wsConfig_SteeringPid.output);
            error = Rover4ws_ErrorCheck(Rover4ws_MotorSpeedControl(ROVER_4WS_CONFIG_MOTOR_0),
                                        Rover4ws_MotorSpeedControl(ROVER_4WS_CONFIG_MOTOR_1),
                                        Rover4ws_MotorSpeedControl(ROVER_4WS_CONFIG_MOTOR_2),
                                        Rover4ws_MotorSpeedControl(ROVER_4WS_CONFIG_MOTOR_3));
        }
    }

    return error;
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

        Rover4ws_CommandedSpeed            = speed;
        Rover4wsConfig_SteeringPid.command = steering_angle;
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

static void Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle)
{
    double radius              = kRover4wsConfig_HalfWheelbase / tan(steering_angle);
    double left_angular_speed  = (speed * (2 - (kRover4wsConfig_Tread / radius))) / kRover4wsConfig_DoubleWheelRadius;
    double right_angular_speed = (speed * (2 + (kRover4wsConfig_Tread / radius))) / kRover4wsConfig_DoubleWheelRadius;

    Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_0].command = left_angular_speed;
    Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_2].command = left_angular_speed;
    Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_1].command = right_angular_speed;
    Rover4wsConfig_MotorsPid[ROVER_4WS_CONFIG_MOTOR_3].command = right_angular_speed;
}

static Rover_Error_t Rover4ws_MotorSpeedControl(const Rover4wsConfig_Motor_t motor)
{
    Rover_Error_t error = ROVER_ERROR_PERIPHERAL;

    if (motor < ROVER_4WS_CONFIG_MOTOR_MAX)
    {
        error = RoverPid_Update(&Rover4wsConfig_MotorsPid[motor], BspEncoderUser_HandleTable[Rover4wsConfig_Encoders[motor]].angular_rate, BspTick_GetMicroseconds());

        if (ROVER_ERROR_NONE == error)
        {
            error = Rover4ws_SetMotorSpeed(motor, Rover4wsConfig_MotorsPid[motor].output);
        }
    }

    return error;
}

static Rover_Error_t Rover4ws_SetMotorSpeed(const Rover4wsConfig_Motor_t motor, const Rover_RadiansPerSecond_t speed)
{
    Rover_Error_t error = ROVER_ERROR_PERIPHERAL;

    if (motor < ROVER_4WS_CONFIG_MOTOR_MAX)
    {
        if (speed < 0.0)
        {
            error = Rover_BspToRoverError(BspMotor_Reverse(&Rover4wsConfig_Motors[motor]));
        }
        else
        {
            error = Rover_BspToRoverError(BspMotor_Forward(&Rover4wsConfig_Motors[motor]));
        }

        if (ROVER_ERROR_NONE == error)
        {
            error = Rover_BspToRoverError(BspMotor_SetSpeed(&Rover4wsConfig_Motors[motor], fabs(speed)));
        }
    }

    return error;
}

static Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle)
{
    Rover_Error_t error = ROVER_ERROR_VALUE;

    double        tan_delta        = tan(steering_angle);
    Rover_Meter_t scaled_wheelbase = kRover4wsConfig_HalfWheelbase * tan_delta;
    Rover_Meter_t offset           = kRover4wsConfig_HalfTread * tan_delta;

    Rover_Radian_t delta_left  = atan(scaled_wheelbase / (kRover4wsConfig_HalfWheelbase - offset));
    Rover_Radian_t delta_right = atan(scaled_wheelbase / (kRover4wsConfig_HalfWheelbase + offset));

    if (Rover4ws_CompareDoubleSigns(delta_left, delta_right))
    {
        error = Rover4ws_BspErrorCheck(BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_0], (ROVER_4WS_WHEEL_OFFSET - delta_left)),
                                       BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_1], (ROVER_4WS_WHEEL_OFFSET - delta_right)),
                                       BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_2], (ROVER_4WS_WHEEL_OFFSET + delta_left)),
                                       BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_3], (ROVER_4WS_WHEEL_OFFSET + delta_right)));
    }

    return error;
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

static inline bool Rover4ws_CompareDoubleSigns(const double value_1, const double value_2)
{
    return !(bool)((*(uint64_t *)(&value_1) & ROVER_4WS_DOUBLE_SIGN_MASK) ^ (*(uint64_t *)(&value_2) & ROVER_4WS_DOUBLE_SIGN_MASK));
}