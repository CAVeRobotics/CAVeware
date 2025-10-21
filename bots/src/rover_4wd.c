#include "rover_4wd.h"

#include <float.h>
#include <math.h>

#include "bsp.h"
#include "bsp_motor.h"

#include "cavebot.h"
#include "cavebot_user.h"

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_Tread       = 0.403225;
static const Bsp_Meter_t kRover4wd_WheelRadius = 0.079375;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_HalfTread         = kRover4wd_Tread / 2;
static const Bsp_Meter_t kRover4wd_DoubleWheelRadius = kRover4wd_WheelRadius * 2;

static Bsp_RadiansPerSecond_t Rover4wd_CommandedWheelSpeedLeft  = 0.0;
static Bsp_RadiansPerSecond_t Rover4wd_CommandedWheelSpeedRight = 0.0;

static Cavebot_Error_t Rover4wd_SetMotorSpeed(const CavebotUser_Motor_t motor, const Bsp_RadiansPerSecond_t speed);
static Cavebot_Error_t Rover4wd_ErrorCheck(const Cavebot_Error_t error_0,
                                           const Cavebot_Error_t error_1,
                                           const Cavebot_Error_t error_2,
                                           const Cavebot_Error_t error_3);
static Cavebot_Error_t Rover4wd_BspErrorCheck(const Bsp_Error_t error_0,
                                              const Bsp_Error_t error_1,
                                              const Bsp_Error_t error_2,
                                              const Bsp_Error_t error_3);

Cavebot_Error_t Rover4wd_Arm(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
    Rover4wd_CommandedWheelSpeedLeft  = 0.0;
    Rover4wd_CommandedWheelSpeedRight = 0.0;

    /* TODO reset PID controller */

    /* TODO enable motor sleep lines */

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4wd_BspErrorCheck(BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    return error;
}

Cavebot_Error_t Rover4wd_Disarm(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    /* TODO disable motor sleep lines */

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4wd_BspErrorCheck(BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    return error;
}

Cavebot_Error_t Rover4wd_Task(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (Cavebot_IsArmed())
    {
        /* TODO sample IMU and encoders */

        /* TODO apply PID control */

        /* TODO replace placeholders with PID outputs */
        error = Rover4wd_ErrorCheck(Rover4wd_SetMotorSpeed(CAVEBOT_USER_MOTOR_0, Rover4wd_CommandedWheelSpeedLeft),
                                    Rover4wd_SetMotorSpeed(CAVEBOT_USER_MOTOR_1, Rover4wd_CommandedWheelSpeedRight),
                                    Rover4wd_SetMotorSpeed(CAVEBOT_USER_MOTOR_2, Rover4wd_CommandedWheelSpeedLeft),
                                    Rover4wd_SetMotorSpeed(CAVEBOT_USER_MOTOR_3, Rover4wd_CommandedWheelSpeedRight));
    }

    return error;
}

Cavebot_Error_t Rover4wd_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (!Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else if (0.0 != speed)
    {
        Bsp_Meter_t wheel_speed_scalar = (kRover4wd_Tread * turn_rate) / speed;
        Rover4wd_CommandedWheelSpeedLeft  = (speed * (2 - wheel_speed_scalar)) / kRover4wd_DoubleWheelRadius;
        Rover4wd_CommandedWheelSpeedRight = (speed * (2 + wheel_speed_scalar)) / kRover4wd_DoubleWheelRadius;
    }
    else if (0.0 != turn_rate)
    {
        Rover4wd_CommandedWheelSpeedRight = (turn_rate * kRover4wd_HalfTread) / kRover4wd_WheelRadius;
        Rover4wd_CommandedWheelSpeedLeft  = -Rover4wd_CommandedWheelSpeedRight;
    }
    else
    {
        Rover4wd_CommandedWheelSpeedLeft  = 0.0;
        Rover4wd_CommandedWheelSpeedRight = 0.0;
    }

    return error;
}

static Cavebot_Error_t Rover4wd_SetMotorSpeed(const CavebotUser_Motor_t motor, const Bsp_RadiansPerSecond_t speed)
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

static Cavebot_Error_t Rover4wd_ErrorCheck(const Cavebot_Error_t error_0,
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

static Cavebot_Error_t Rover4wd_BspErrorCheck(const Bsp_Error_t error_0,
                                              const Bsp_Error_t error_1,
                                              const Bsp_Error_t error_2,
                                              const Bsp_Error_t error_3)
{
    Cavebot_Error_t rover_error_0 = Cavebot_BspToCavebotError(error_0);
    Cavebot_Error_t rover_error_1 = Cavebot_BspToCavebotError(error_1);
    Cavebot_Error_t rover_error_2 = Cavebot_BspToCavebotError(error_2);
    Cavebot_Error_t rover_error_3 = Cavebot_BspToCavebotError(error_3);

    return Rover4wd_ErrorCheck(rover_error_0, rover_error_1, rover_error_2, rover_error_3);
}