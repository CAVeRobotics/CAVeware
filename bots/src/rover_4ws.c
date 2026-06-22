#include "rover_4ws.h"

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
#include "pid.h"

#define ROVER_4WS_WHEEL_OFFSET (double)(3.14159265358979323846 / 2.0)

static const char *const kRover4ws_LogTag = "ROVER 4WS";

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4ws_Tread       = 0.493800;
static const Bsp_Meter_t kRover4ws_Wheelbase   = 0.466028;
static const Bsp_Meter_t kRover4ws_WheelRadius = 0.080000;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4ws_HalfTread     = kRover4ws_Tread / 2;
static const Bsp_Meter_t kRover4ws_HalfWheelbase = kRover4ws_Wheelbase / 2;
static const Bsp_Meter_t kRover4ws_WheelDiameter = kRover4ws_WheelRadius * 2;

/* TODO CVW-21 read from config */
static Pid_Handle_t Rover4ws_SteeringPid = {
    0
};

/* TODO CVW-21 read gains, rate limit, enabled, minimum, maxmimum from config */
static Pid_Handle_t Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_MAX] = {
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

static Bsp_MetersPerSecond_t Rover4ws_CommandedSpeed = 0.0;

static void Rover4ws_SetSpeed(const Bsp_MetersPerSecond_t speed, const Bsp_Radian_t steering_angle);
static void Rover4ws_MotorSpeedControl(const CavebotUser_Motor_t motor);
static void Rover4ws_SetSteeringAngle(const Bsp_Radian_t steering_angle);
static Bsp_Error_t Rover4ws_ErrorCheck(const Bsp_Error_t error_0,
                                       const Bsp_Error_t error_1,
                                       const Bsp_Error_t error_2,
                                       const Bsp_Error_t error_3);

bool Rover4ws_Arm(void)
{
    Pid_Reset(&Rover4ws_SteeringPid);
    Pid_Reset(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_0]);
    Pid_Reset(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_2]);
    Pid_Reset(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_1]);
    Pid_Reset(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_3]);

    Rover4ws_SteeringPid.command                     = 0.0;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_0].command = 0.0;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_1].command = 0.0;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_2].command = 0.0;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_3].command = 0.0;

    bool        armed = false;
    Bsp_Error_t error = Rover4ws_ErrorCheck(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_0_SLEEP, BSP_GPIO_STATE_SET),
                                            BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_1_SLEEP, BSP_GPIO_STATE_SET),
                                            BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_2_SLEEP, BSP_GPIO_STATE_SET),
                                            BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_3_SLEEP, BSP_GPIO_STATE_SET));

    if (BSP_ERROR_NONE == error)
    {
        error = Rover4ws_ErrorCheck(BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                    BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                    BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                    BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_MOTOR, error);
        }
    }

    if (BSP_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_0]),
                                       BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_2]),
                                       BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_1]),
                                       BspServo_Start(&CavebotUser_Servos[CAVEBOT_USER_SERVO_3]));

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_SERVO, error);
        }
    }

    if (BSP_ERROR_NONE == error)
    {
        armed = true;
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kRover4ws_LogTag, "Failed to arm with error %s", Bsp_ErrorToString(error));
    }

    return armed;
}

bool Rover4ws_Disarm(void)
{
    bool        disarmed = false;
    Bsp_Error_t error    = Rover4ws_ErrorCheck(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_0_SLEEP, BSP_GPIO_STATE_RESET),
                                               BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_1_SLEEP, BSP_GPIO_STATE_RESET),
                                               BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_2_SLEEP, BSP_GPIO_STATE_RESET),
                                               BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_3_SLEEP, BSP_GPIO_STATE_RESET));

    if (BSP_ERROR_NONE == error)
    {
        error = Rover4ws_ErrorCheck(BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                    BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                    BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                    BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_MOTOR, error);
        }
    }

    if (BSP_ERROR_NONE == error)
    {
        error = Rover4ws_BspErrorCheck(BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_0]),
                                       BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_2]),
                                       BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_1]),
                                       BspServo_Stop(&CavebotUser_Servos[CAVEBOT_USER_SERVO_3]));

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_SERVO, error);
        }
    }

    if (BSP_ERROR_NONE == error)
    {
        disarmed = true;
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kRover4ws_LogTag, "Failed to disarm with error %s", Bsp_ErrorToString(error));
    }

    return disarmed;
}

void Rover4ws_EnableSpeedControl(void)
{
    Pid_Enable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_0]);
    Pid_Enable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_1]);
    Pid_Enable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_2]);
    Pid_Enable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_3]);

    BSP_LOGGER_LOG_DEBUG(kRover4ws_LogTag, "Speed control enabled");
}

void Rover4ws_DisableSpeedControl(void)
{
    Pid_Disable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_0]);
    Pid_Disable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_1]);
    Pid_Disable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_2]);
    Pid_Disable(&Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_3]);

    BSP_LOGGER_LOG_DEBUG(kRover4ws_LogTag, "Speed control disabled");
}

void Rover4ws_EnableSteeringControl(void)
{
    Pid_Enable(&Rover4ws_SteeringPid);

    BSP_LOGGER_LOG_DEBUG(kRover4ws_LogTag, "Steering control enabled");
}

void Rover4ws_DisableSteeringControl(void)
{
    Pid_Disable(&Rover4ws_SteeringPid);

    BSP_LOGGER_LOG_DEBUG(kRover4ws_LogTag, "Steering control disabled");
}

void Rover4ws_Run(void)
{
    if (Cavebot_IsArmed())
    {
        /* TODO SD-126 test with steering control coupled and decoupled from wheel speed control */
        Pid_Update(&Rover4ws_SteeringPid, CavebotUser_Gyroscope.reading.z);
        Rover4ws_SetSteeringAngle(Rover4ws_SteeringPid.output);

        Rover4ws_SetSpeed(Rover4ws_CommandedSpeed, Rover4ws_SteeringPid.output);
        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_0);
        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_1);
        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_2);
        Rover4ws_MotorSpeedControl(CAVEBOT_USER_MOTOR_3);
    }
}

void Rover4ws_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    BSP_LOGGER_LOG_VERBOSE(kRover4ws_LogTag, "Received speed: %lf, turn rate: %lf", speed, turn_rate);

    if ((0.0 != speed) && (Cavebot_IsArmed()))
    {
        Bsp_Radian_t steering_angle = atan((turn_rate * kRover4ws_HalfWheelbase) / speed);

        Rover4ws_CommandedSpeed      = speed;
        Rover4ws_SteeringPid.command = steering_angle;
    }
}

static void Rover4ws_SetSpeed(const Bsp_MetersPerSecond_t speed, const Bsp_Radian_t steering_angle)
{
    double radius              = kRover4ws_HalfWheelbase / tan(steering_angle);
    double left_angular_speed  = (speed * (2.0 - (kRover4ws_Tread / radius))) / kRover4ws_WheelDiameter;
    double right_angular_speed = (speed * (2.0 + (kRover4ws_Tread / radius))) / kRover4ws_WheelDiameter;

    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_0].command = left_angular_speed;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_2].command = left_angular_speed;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_1].command = right_angular_speed;
    Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_3].command = right_angular_speed;

    BSP_LOGGER_LOG_VERBOSE(kRover4ws_LogTag,
                           "Set wheel speeds %d:%lf, %d:%lf, %d:%lf, %d:%lf",
                           CAVEBOT_USER_MOTOR_0, Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_0].command,
                           CAVEBOT_USER_MOTOR_1, Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_1].command,
                           CAVEBOT_USER_MOTOR_2, Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_2].command,
                           CAVEBOT_USER_MOTOR_3, Rover4ws_MotorsPid[CAVEBOT_USER_MOTOR_3].command);
}

static void Rover4ws_MotorSpeedControl(const CavebotUser_Motor_t motor)
{
    if (motor < CAVEBOT_USER_MOTOR_MAX)
    {
        Bsp_Error_t error = BSP_ERROR_NONE;

        Pid_Update(&Rover4ws_MotorsPid[motor], BspEncoderUser_HandleTable[CavebotUser_Encoders[motor]].angular_rate);

        if (fabs(Rover4ws_MotorsPid[motor].command) <= DBL_MIN)
        {
            Pid_Reset(&Rover4ws_MotorsPid[motor]);
        }
        else if (Rover4ws_MotorsPid[motor].output < 0.0)
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
        else if (Rover4ws_MotorsPid[motor].enabled)
        {
            /* TODO CVW-70 test active braking if output is zero */
            error = BspMotor_SetDutyCycle(&CavebotUser_Motors[motor], fabs(Rover4ws_MotorsPid[motor].output));
        }
        else
        {
            error = BspMotor_SetSpeed(&CavebotUser_Motors[motor], fabs(Rover4ws_MotorsPid[motor].output));
        }

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_MOTOR, error);
            BSP_LOGGER_LOG_ERROR(kRover4ws_LogTag, "Failed motor speed control with error %s", Bsp_ErrorToString(error));
        }
    }
}

static void Rover4ws_SetSteeringAngle(const Bsp_Radian_t steering_angle)
{
    double      tan_delta        = tan(steering_angle);
    Bsp_Meter_t scaled_wheelbase = kRover4ws_HalfWheelbase * tan_delta;
    Bsp_Meter_t offset           = kRover4ws_HalfTread * tan_delta;

    Bsp_Radian_t delta_left  = atan(scaled_wheelbase / (kRover4ws_HalfWheelbase - offset));
    Bsp_Radian_t delta_right = atan(scaled_wheelbase / (kRover4ws_HalfWheelbase + offset));

    if (Bsp_CompareDoubleSigns(&delta_left, &delta_right))
    {
        const Bsp_Error_t error = Rover4ws_ErrorCheck(BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_0], (ROVER_4WS_WHEEL_OFFSET - delta_left)),
                                                      BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_1], (ROVER_4WS_WHEEL_OFFSET - delta_right)),
                                                      BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_2], (ROVER_4WS_WHEEL_OFFSET + delta_left)),
                                                      BspServo_SetAngle(&CavebotUser_Servos[CAVEBOT_USER_SERVO_3], (ROVER_4WS_WHEEL_OFFSET + delta_right)));

        if (BSP_ERROR_NONE != error)
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_SERVO, error);
            BSP_LOGGER_LOG_ERROR(kRover4ws_LogTag, "Failed to set steering angle with error %s", Bsp_ErrorToString(error));
        }
    }
}

static Bsp_Error_t Rover4ws_ErrorCheck(const Bsp_Error_t error_0,
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
