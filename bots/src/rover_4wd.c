#include "rover_4wd.h"

#include <float.h>
#include <math.h>

#include "bsp.h"
#include "bsp_encoder_user.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_motor.h"
#include "bsp_tick.h"

#include "cavebot.h"
#include "cavebot_user.h"
#include "rover_4wd_ekf.h"

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_Tread       = 0.403225;
static const Bsp_Meter_t kRover4wd_WheelRadius = 0.079375;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_HalfTread     = kRover4wd_Tread / 2;
static const Bsp_Meter_t kRover4wd_WheelDiameter = kRover4wd_WheelRadius * 2;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kRover4wd_MetersPerPulse = (kRover4wd_WheelDiameter * BSP_PI) / 753.2;

static Bsp_Meter_t       Rover4wd_DistanceLeft  = 0.0;
static Bsp_Meter_t       Rover4wd_DistanceRight = 0.0;
static Bsp_Microsecond_t Rover4wd_PredictTick   = 0U;
static Bsp_Microsecond_t Rover4wd_UpdateTick    = 0U;

/* TODO CVW-21 read gains, rate limit, enabled, minimum, maxmimum from config */
static CavebotPid_Handle_t Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_MAX] = {
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
static Cavebot_Error_t Rover4wd_MotorSpeedControl(const CavebotUser_Motor_t motor);
static Cavebot_Error_t Rover4wd_ErrorCheck(const Cavebot_Error_t error_0,
                                           const Cavebot_Error_t error_1,
                                           const Cavebot_Error_t error_2,
                                           const Cavebot_Error_t error_3);
static Cavebot_Error_t Rover4wd_BspErrorCheck(const Bsp_Error_t error_0,
                                              const Bsp_Error_t error_1,
                                              const Bsp_Error_t error_2,
                                              const Bsp_Error_t error_3);

Cavebot_Error_t Rover4wd_Initialize(void)
{
    Rover4wdEkf_Initialize();

    return CAVEBOT_ERROR_NONE;
}

Cavebot_Error_t Rover4wd_Arm(void)
{
    Cavebot_Error_t error = Rover4wd_ErrorCheck(CavebotPid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0]),
                                                CavebotPid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2]),
                                                CavebotPid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1]),
                                                CavebotPid_Reset(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3]));

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4wd_BspErrorCheck(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_0_SLEEP, BSP_GPIO_STATE_SET),
                                       BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_1_SLEEP, BSP_GPIO_STATE_SET),
                                       BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_2_SLEEP, BSP_GPIO_STATE_SET),
                                       BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_3_SLEEP, BSP_GPIO_STATE_SET));
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4wd_BspErrorCheck(BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                       BspMotor_Start(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0].command = 0.0;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1].command = 0.0;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2].command = 0.0;
    Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3].command = 0.0;

    return error;
}

Cavebot_Error_t Rover4wd_Disarm(void)
{
    Cavebot_Error_t error = Rover4wd_BspErrorCheck(BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_0_SLEEP, BSP_GPIO_STATE_RESET),
                                                   BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_1_SLEEP, BSP_GPIO_STATE_RESET),
                                                   BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_2_SLEEP, BSP_GPIO_STATE_RESET),
                                                   BspGpio_Write(BSP_GPIO_USER_PIN_MOTOR_3_SLEEP, BSP_GPIO_STATE_RESET));

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4wd_BspErrorCheck(BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_0]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_1]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_2]),
                                       BspMotor_Stop(&CavebotUser_Motors[CAVEBOT_USER_MOTOR_3]));
    }

    return error;
}

Cavebot_Error_t Rover4wd_EnableSpeedControl(void)
{
    return Rover4wd_ErrorCheck(CavebotPid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0]),
                               CavebotPid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1]),
                               CavebotPid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2]),
                               CavebotPid_Enable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3]));
}

Cavebot_Error_t Rover4wd_DisableSpeedControl(void)
{
    return Rover4wd_ErrorCheck(CavebotPid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_0]),
                               CavebotPid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_1]),
                               CavebotPid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_2]),
                               CavebotPid_Disable(&Rover4wd_MotorsPid[CAVEBOT_USER_MOTOR_3]));
}

Cavebot_Error_t Rover4wd_Task(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    Rover4wd_EstimatePose();

    if (Cavebot_IsArmed())
    {
        error = Rover4wd_ErrorCheck(Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_0),
                                    Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_1),
                                    Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_2),
                                    Rover4wd_MotorSpeedControl(CAVEBOT_USER_MOTOR_3));
    }

    return error;
}

Cavebot_Error_t Rover4wd_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    Cavebot_Error_t        error                       = CAVEBOT_ERROR_NONE;
    Bsp_RadiansPerSecond_t commanded_wheel_speed_left  = 0.0;
    Bsp_RadiansPerSecond_t commanded_wheel_speed_right = 0.0;

    if (!Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
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

    return error;
}

Cavebot_Pose_t Rover4wd_GetPose(void)
{
    return (Cavebot_Pose_t){
               .x       = Rover4wdEfk_GetX(),
               .y       = Rover4wdEfk_GetY(),
               .heading = Rover4wdEfk_GetHeading(),
    };
}

static void Rover4wd_EstimatePose(void)
{
    const Bsp_Meter_t distance_left  = (((double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses + (double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses) / 2.0) * kRover4wd_MetersPerPulse;
    const Bsp_Meter_t distance_right = (((double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses + (double)BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses) / 2.0) * kRover4wd_MetersPerPulse;
    const Bsp_Meter_t delta_left     = distance_left - Rover4wd_DistanceLeft;
    const Bsp_Meter_t delta_right    = distance_right - Rover4wd_DistanceRight;
    Rover4wd_DistanceLeft  = distance_left;
    Rover4wd_DistanceRight = distance_right;

    const Bsp_Microsecond_t tick               = BspTick_GetMicroseconds();
    const Bsp_Second_t      delta_time_predict = BspTick_GetElapsedMicroseconds(Rover4wd_PredictTick, tick);
    const Bsp_Second_t      delta_time_update  = BspTick_GetElapsedMicroseconds(Rover4wd_UpdateTick, tick);

    if (delta_time_predict >= 0.001)
    {
        Rover4wdEkf_Predict(delta_time_predict, CavebotUser_Accelerometer.reading.x, CavebotUser_Accelerometer.reading.y, CavebotUser_Gyroscope.reading.z);
        Rover4wd_PredictTick = tick;
    }

    if (delta_time_update >= 0.01)
    {
        Rover4wdEkf_Update(delta_time_update, CavebotUser_Gyroscope.reading.z, delta_left, delta_right);
        Rover4wd_UpdateTick = tick;
    }
}

static Cavebot_Error_t Rover4wd_MotorSpeedControl(const CavebotUser_Motor_t motor)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_PERIPHERAL;

    if (motor < CAVEBOT_USER_MOTOR_MAX)
    {
        error = CavebotPid_Update(&Rover4wd_MotorsPid[motor], BspEncoderUser_HandleTable[CavebotUser_Encoders[motor]].angular_rate);

        if (CAVEBOT_ERROR_NONE != error)
        {
        }
        else if (fabs(Rover4wd_MotorsPid[motor].command) <= DBL_MIN)
        {
            error = CavebotPid_Reset(&Rover4wd_MotorsPid[motor]);
        }
        else if (Rover4wd_MotorsPid[motor].output < 0.0)
        {
            error = Cavebot_BspToCavebotError(BspMotor_Reverse(&CavebotUser_Motors[motor]));
        }
        else
        {
            error = Cavebot_BspToCavebotError(BspMotor_Forward(&CavebotUser_Motors[motor]));
        }

        if (CAVEBOT_ERROR_NONE != error)
        {
        }
        else if (Rover4wd_MotorsPid[motor].enabled)
        {
            /* TODO CVW-70 test active braking if output is zero */
            error = Cavebot_BspToCavebotError(BspMotor_SetDutyCycle(&CavebotUser_Motors[motor], fabs(Rover4wd_MotorsPid[motor].output)));
        }
        else
        {
            error = Cavebot_BspToCavebotError(BspMotor_SetSpeed(&CavebotUser_Motors[motor], fabs(Rover4wd_MotorsPid[motor].output)));
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