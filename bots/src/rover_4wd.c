#include "rover_4wd.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "bsp.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio.h"
#include "bsp_logger.h"
#include "bsp_gpio_user.h"
#include "bsp_motor.h"
#include "bsp_tick.h"

#include "cavebot.h"
#include "cavebot_user.h"
#include "fault_handler.h"

static const char *const kRover4wd_LogTag = "ROVER 4WD";

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_Tread       = 0.403225;
static const Bsp_Meter_t kRover4wd_WheelRadius = 0.079375;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_HalfTread     = kRover4wd_Tread / 2;
static const Bsp_Meter_t kRover4wd_WheelDiameter = kRover4wd_WheelRadius * 2;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_MetersPerPulse = (kRover4wd_WheelDiameter * BSP_PI) / 753.2;

static Bsp_Meter_t            Rover4wd_DistanceLeft     = 0.0;
static Bsp_Meter_t            Rover4wd_DistanceRight    = 0.0;
static Bsp_Microsecond_t      Rover4wd_Tick             = 0U;
static const double           kRover4wd_GyroscopeWeight = 0.9; /* TODO CVW-21 read from config */
static const double           kRover4wd_WheelWeight     = 1.0 - kRover4wd_GyroscopeWeight;
static Cavebot_Pose_t         Rover4wd_Pose             = {
    .x       = 0.0,
    .y       = 0.0,
    .heading = 0.0,
};
static Bsp_MetersPerSecond_t  Rover4wd_LinearVelocity  = 0.0;
static Bsp_RadiansPerSecond_t Rover4wd_AngularVelocity = 0.0;

/* TODO CVW-21 read gains, rate limit, enabled, minimum, maxmimum from config */
static Pid_Handle_t Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_MAX] = {
    [CAVEBOT_USER_MOTOR_0] = {
        .kp               = 0.0241,
        .ki               = 0.24132,
        .kd               = 0.000482,
        .kff              = 0.0,
        .rate_limit       = 100.0,
        .integral         = 0.0,
        .command          = 0.0,
        .error            = 0.0,
        .output           = 0.0,
        .previous_tick    = 0U,
        .enabled          = true,
        .integral_enabled = true,
        .minimum          = -1.0,
        .maximum          = 1.0
    },
    [CAVEBOT_USER_MOTOR_1] = {
        .kp               = 0.0241,
        .ki               = 0.24132,
        .kd               = 0.000482,
        .kff              = 0.0,
        .rate_limit       = 100.0,
        .integral         = 0.0,
        .command          = 0.0,
        .error            = 0.0,
        .output           = 0.0,
        .previous_tick    = 0U,
        .enabled          = true,
        .integral_enabled = true,
        .minimum          = -1.0,
        .maximum          = 1.0,
    },
    [CAVEBOT_USER_MOTOR_2] = {
        .kp               = 0.020673,
        .ki               = 0.20673,
        .kd               = 0.000413,
        .kff              = 0.0,
        .rate_limit       = 100.0,
        .integral         = 0.0,
        .command          = 0.0,
        .error            = 0.0,
        .output           = 0.0,
        .previous_tick    = 0U,
        .enabled          = true,
        .integral_enabled = true,
        .minimum          = -1.0,
        .maximum          = 1.0,
    },
    [CAVEBOT_USER_MOTOR_3] = {
        .kp               = 0.020614,
        .ki               = 0.20614,
        .kd               = 0.000412,
        .kff              = 0.0,
        .rate_limit       = 100.0,
        .integral         = 0.0,
        .command          = 0.0,
        .error            = 0.0,
        .output           = 0.0,
        .previous_tick    = 0U,
        .enabled          = true,
        .integral_enabled = true,
        .minimum          = -1.0,
        .maximum          = 1.0,
    }
};

static void Rover4wd_EstimatePose(void);
static void Rover4wd_MotorSpeedControl(const CavebotUser_Motor_t motor);
static Bsp_Error_t Rover4wd_ErrorCheck(const Bsp_Error_t error_0,
                                       const Bsp_Error_t error_1,
                                       const Bsp_Error_t error_2,
                                       const Bsp_Error_t error_3);

bool Rover4wd_Arm(void)
{
    Pid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0]);
    Pid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2]);
    Pid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1]);
    Pid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3]);

    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0].command = 0.0;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1].command = 0.0;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2].command = 0.0;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3].command = 0.0;

    bool        armed = false;
    Bsp_Error_t error = Rover4wd_ErrorCheck(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_0_SLEEP, BSP_GPIO_STATE_SET),
                                            BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_1_SLEEP, BSP_GPIO_STATE_SET),
                                            BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_2_SLEEP, BSP_GPIO_STATE_SET),
                                            BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_3_SLEEP, BSP_GPIO_STATE_SET));

    if (BSP_ERROR_NONE == error)
    {
        error = Rover4wd_ErrorCheck(BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                    BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                    BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                    BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    if (BSP_ERROR_NONE == error)
    {
        armed = true;
    }
    else
    {
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_MOTOR, error);
        BSP_LOGGER_LOG_ERROR(kRover4wd_LogTag, "Failed to arm with error %s", Bsp_ErrorToString(error));
    }

    return armed;
}

bool Rover4wd_Disarm(void)
{
    bool        disarmed = false;
    Bsp_Error_t error    = Rover4wd_ErrorCheck(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_0_SLEEP, BSP_GPIO_STATE_RESET),
                                               BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_1_SLEEP, BSP_GPIO_STATE_RESET),
                                               BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_2_SLEEP, BSP_GPIO_STATE_RESET),
                                               BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_3_SLEEP, BSP_GPIO_STATE_RESET));

    if (BSP_ERROR_NONE == error)
    {
        error = Rover4wd_ErrorCheck(BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                    BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                    BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                    BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    if (BSP_ERROR_NONE == error)
    {
        disarmed = true;
    }
    else
    {
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_MOTOR, error);
        BSP_LOGGER_LOG_ERROR(kRover4wd_LogTag, "Failed to disarm with error %s", Bsp_ErrorToString(error));
    }

    return disarmed;
}

void Rover4wd_EnableSpeedControl(void)
{
    Pid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0]);
    Pid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1]);
    Pid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2]);
    Pid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3]);

    BSP_LOGGER_LOG_DEBUG(kRover4wd_LogTag, "Speed control enabled");
}

void Rover4wd_DisableSpeedControl(void)
{
    Pid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0]);
    Pid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1]);
    Pid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2]);
    Pid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3]);

    BSP_LOGGER_LOG_DEBUG(kRover4wd_LogTag, "Speed control disabled");
}

void Rover4wd_Run(void)
{
    Rover4wd_EstimatePose();

    if (Cavebot_IsArmed())
    {
        Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_0);
        Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_1);
        Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_2);
        Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_3);
    }
}

void Rover4wd_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    Bsp_RadiansPerSecond_t commanded_wheel_speed_left  = 0.0;
    Bsp_RadiansPerSecond_t commanded_wheel_speed_right = 0.0;

    BSP_LOGGER_LOG_VERBOSE(kRover4wd_LogTag, "Received speed: %lf, turn rate: %lf", speed, turn_rate);

    if (!Cavebot_IsArmed())
    {
    }
    else if (0.0 != speed)
    {
        const Bsp_Meter_t           wheel_speed_scalar = kRover4wd_Tread * turn_rate;
        const Bsp_MetersPerSecond_t double_speed       = speed * 2.0;
        commanded_wheel_speed_left  = (double_speed - wheel_speed_scalar) / kRover4wd_WheelDiameter;
        commanded_wheel_speed_right = (double_speed + wheel_speed_scalar) / kRover4wd_WheelDiameter;
    }
    else if (0.0 != turn_rate)
    {
        commanded_wheel_speed_right = (turn_rate * kRover4wd_HalfTread) / kRover4wd_WheelRadius;
        commanded_wheel_speed_left  = -commanded_wheel_speed_right;
    }
    else
    {
        commanded_wheel_speed_left  = 0.0;
        commanded_wheel_speed_right = 0.0;
    }

    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0].command = commanded_wheel_speed_left;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1].command = commanded_wheel_speed_right;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2].command = commanded_wheel_speed_left;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3].command = commanded_wheel_speed_right;

    BSP_LOGGER_LOG_VERBOSE(kRover4wd_LogTag,
                           "Set wheel speeds %d:%lf, %d:%lf, %d:%lf, %d:%lf",
                           CAVEBOT_USER_MOTOR_0, Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0].command,
                           CAVEBOT_USER_MOTOR_1, Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1].command,
                           CAVEBOT_USER_MOTOR_2, Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2].command,
                           CAVEBOT_USER_MOTOR_3, Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3].command);
}

Cavebot_Pose_t Rover4wd_GetPose(void)
{
    return Rover4wd_Pose;
}

void Rover4wd_SetPose(const Cavebot_Pose_t *const pose)
{
    if (NULL != pose)
    {
        Rover4wd_Pose = *pose;
        Rover4wd_Tick = BspTick_GetMicroseconds();
    }
}

Bsp_MetersPerSecond_t Rover4wd_GetLinearVelocity(void)
{
    return Rover4wd_LinearVelocity;
}

Bsp_RadiansPerSecond_t Rover4wd_GetAngularVelocity(void)
{
    return Rover4wd_AngularVelocity;
}

static void Rover4wd_EstimatePose(void)
{
    /* Wheel odometry */
    const Bsp_Meter_t  distance_left        = (((double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses + (double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses) / 2.0) * kRover4wd_MetersPerPulse;
    const Bsp_Meter_t  distance_right       = (((double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses + (double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses) / 2.0) * kRover4wd_MetersPerPulse;
    const Bsp_Meter_t  delta_left           = distance_left - Rover4wd_DistanceLeft;
    const Bsp_Meter_t  delta_right          = distance_right - Rover4wd_DistanceRight;
    const Bsp_Meter_t  delta_center         = (delta_left + delta_right) / 2.0;
    const Bsp_Radian_t delta_heading_wheels = (delta_right - delta_left) / kRover4wd_Tread; /* TODO explain */
    Rover4wd_DistanceLeft  = distance_left;
    Rover4wd_DistanceRight = distance_right;

    /* Gyroscope heading */
    const Bsp_Millisecond_t tick                    = BspTick_GetMicroseconds();
    const Bsp_Second_t      delta_time              = BspTick_GetElapsedMicroseconds(Rover4wd_Tick, tick);
    Bsp_Radian_t            delta_heading_gyroscope = 0.0;
    if ((fabs(delta_left) > 0.001) || (fabs(delta_right) > 0.001))
    {
        delta_heading_gyroscope = CavebotUser_Gyroscope.reading.z * delta_time;
    }
    Rover4wd_Tick = tick;

    /* Fused heading */
    const Bsp_Radian_t delta_heading = (delta_heading_gyroscope * kRover4wd_GyroscopeWeight) + (delta_heading_wheels * kRover4wd_WheelWeight);
    const Bsp_Radian_t heading       = Rover4wd_Pose.heading + (delta_heading / 2.0); /* Use the average heading for more accurate integration */

    /* Update pose */
    Rover4wd_Pose.x       += delta_center * cos(heading);
    Rover4wd_Pose.y       += delta_center * sin(heading);
    Rover4wd_Pose.heading += delta_heading;
    Rover4wd_Pose.heading  = atan2(sin(Rover4wd_Pose.heading), cos(Rover4wd_Pose.heading)); /* Normalize to [-pi, pi] */

    /* Update velocities */
    if (delta_time > 0.0)
    {
        Rover4wd_LinearVelocity  = delta_center / delta_time;
        Rover4wd_AngularVelocity = delta_heading / delta_time;
    }
}

static void Rover4wd_MotorSpeedControl(const CavebotUser_Motor_t motor)
{
    if (motor < CAVEBOT_USER_MOTOR_MAX)
    {
        Bsp_Error_t error = BSP_ERROR_NONE;

        Pid_Update(&Rover4wd_MotorsPid[motor], BspEncoderUser_HandleTable[CavebotUser_Encoders[motor]].angular_rate);

        if (fabs(Rover4wd_MotorsPid[motor].command) <= DBL_MIN)
        {
            Pid_Reset(&Rover4wd_MotorsPid[motor]);
        }
        else if (Rover4wd_MotorsPid[motor].output < 0.0)
        {
            error = BspMotor_Reverse(&CavebotUser_Motors[motor]);
        }
        else
        {
            error = BspMotor_Forward(&CavebotUser_Motors[motor]);
        }

        if (BSP_ERROR_NONE != error)
        {
        }
        else if (Rover4wd_MotorsPid[motor].enabled)
        {
            /* TODO CVW-70 test active braking if output is zero */
            error = BspMotor_SetDutyCycle(&CavebotUser_Motors[motor], fabs(Rover4wd_MotorsPid[motor].output));
        }
        else
        {
            error = BspMotor_SetSpeed(&CavebotUser_Motors[motor], fabs(Rover4wd_MotorsPid[motor].output));
        }

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_MOTOR, error);
            BSP_LOGGER_LOG_ERROR(kRover4wd_LogTag, "Failed motor speed control with error %s", Bsp_ErrorToString(error));
        }
    }
}

static Bsp_Error_t Rover4wd_ErrorCheck(const Bsp_Error_t error_0,
                                       const Bsp_Error_t error_1,
                                       const Bsp_Error_t error_2,
                                       const Bsp_Error_t error_3)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (BSP_ERROR_NONE != error_0)
    {
        error = error_0;
    }
    else if (BSP_ERROR_NONE != error_1)
    {
        error = error_1;
    }
    else if (BSP_ERROR_NONE != error_2)
    {
        error = error_2;
    }
    else if (BSP_ERROR_NONE != error_3)
    {
        error = error_3;
    }

    return error;
}