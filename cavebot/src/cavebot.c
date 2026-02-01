#include "cavebot.h"

#include <math.h>
#include <stdbool.h>

#include "bsp.h"
#include "bsp_tick.h"
#include "bsp_logger.h"

#include "accelerometer.h"
#include "gyroscope.h"

#include "cavebot_cavetalk.h"
#include "cavebot_motion_profile.h"
#include "cavebot_user.h"
#include "cavebot_version.h"
#ifdef ROVER_4WD
#include "rover_4wd.h"
#endif
#ifdef ROVER_4WS
#include "rover_4ws.h"
#endif

#define CAVEBOT_LOOP_LOG_PERIOD (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)

static const char *      kCavebot_LogTag                = "CAVEBOT";
static Cavebot_Bot_t     Cavebot_Bot                    = CAVEBOT_BOT_4WD;
static Cavebot_Mode_t Cavebot_Mode = CAVEBOT_MODE_DISARMED;
static Cavebot_Pose_t Cavebot_Waypoint = {
    .x = 0.0,
    .y = 0.0,
    .heading = 0.0
};
static bool Cavebot_HasWaypoint = false; /* TODO replace with queue */

/* TODO delete relative move */
static Bsp_Meter_t       Cavebot_RelativeMovePosition   = 0.0;
static Bsp_Radian_t      Cavebot_RelativeMovePose       = 0.0;
static Bsp_Millisecond_t Cavebot_RelativeMoveTick       = 0U;
static bool              Cavebot_RelativeMovingPosition = false;
static bool              Cavebot_RelativeMovingPose     = false;

/* TODO CVW-21 read from config */
const Bsp_Meter_t kCavebot_MetersPerTick = (0.079375 * 2 * 3.14159265) / 753.2;

static Cavebot_Error_t Cavebot_Initialize(void);
static void Cavebot_Task(void);
static void Cavebot_MeasureLoopRate(void);

int main(void)
{
    Bsp_Initialize();

    /* Immediately print out build info in case there is a problem starting BSP tick */
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build branch: %s", CAVEBOT_GIT_BRANCH);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build commit: %s", CAVEBOT_GIT_COMMIT_HASH);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build tag: %s", CAVEBOT_GIT_TAG);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build status: %s", CAVEBOT_GIT_DIRTY);

    if (BSP_ERROR_NONE != BspTick_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start BSP Tick");
    }
    else if (CAVEBOT_ERROR_NONE != Cavebot_Initialize())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to initialize");
    }
    else if (CAVE_TALK_ERROR_NONE != CavebotCaveTalk_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start CAVeTalk");
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Initialized");

        // Bsp_Millisecond_t last            = BspTick_GetTick();
        Cavebot_RelativeMoveTick = BspTick_GetTick();
        // Rover4wd_DisableSpeedControl();
        while (true)
        {
            CavebotUser_SensorTask();
            Cavebot_Task();
            CavebotUser_Task();
            CavebotCaveTalk_Task();
            Cavebot_MeasureLoopRate();
        }
    }

    return 0;
}

Cavebot_Error_t Cavebot_BspToCavebotError(const Bsp_Error_t bsp_error)
{
    Cavebot_Error_t cavebot_error = CAVEBOT_ERROR_NONE;

    switch (bsp_error)
    {
    case BSP_ERROR_NONE:
        break;
    case BSP_ERROR_NULL:
        cavebot_error = CAVEBOT_ERROR_NULL;
        break;
    case BSP_ERROR_PERIPHERAL:
        cavebot_error = CAVEBOT_ERROR_PERIPHERAL;
        break;
    case BSP_ERROR_VALUE:
        cavebot_error = CAVEBOT_ERROR_VALUE;
        break;
    case BSP_ERROR_HAL:
    case BSP_ERROR_BUSY:
    case BSP_ERROR_TIMEOUT:
    default:
        cavebot_error = CAVEBOT_ERROR_BSP;
        break;
    }

    return cavebot_error;
}

Cavebot_Error_t Cavebot_Arm(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_BOT;

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        error = Rover4ws_Arm();
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        error = Rover4wd_Arm();
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        Cavebot_Armed = true;

        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Armed", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to arm with error %d", (int)error);
    }

    return error;
}

Cavebot_Error_t Cavebot_Disarm(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_BOT;

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        error = Rover4ws_Disarm();
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        error = Rover4wd_Disarm();
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        Cavebot_Armed = false;

        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Dearmed", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to dearm with error %d", (int)error);
    }

    return error;
}

bool Cavebot_IsArmed(void)
{
    bool armed = false;

    switch (Cavebot_Mode)
    {
    case CAVEBOT_MODE_ARMED_MANUAL:
    case CAVEBOT_MODE_ARMED_AUTO:
        armed = true;
        break;
    default:
        break;
    }

    return armed;
}

Cavebot_Error_t Cavebot_SetAuto(const bool set_auto)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (!Cavebot_IsArmed())
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else if (set_auto)
    {
        Cavebot_Mode = CAVEBOT_MODE_ARMED_AUTO;
    }
    else
    {
        Cavebot_Mode = CAVEBOT_MODE_ARMED_MANUAL;
        error = Cavebot_Drive(0.0, 0.0);
    }

    return error;
}

Cavebot_Error_t Cavebot_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_BOT;

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        error = Rover4ws_Drive(speed, turn_rate);
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        error = Rover4wd_Drive(speed, turn_rate);
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    if (CAVEBOT_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_WARNING(kCavebot_LogTag, "Failed to set speed %lf m/s and turn rate %lf rad/s with error %d", speed, turn_rate, (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_VERBOSE(kCavebot_LogTag, "Set speed %lf m/s and turn rate %lf rad/s", speed, turn_rate);
    }

    return error;
}

Cavebot_Pose_t Cavebot_GetPose(void)
{
    Cavebot_Pose_t pose = {
        .x       = 0.0,
        .y       = 0.0,
        .heading = 0.0,
    };

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        /* TODO */
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        pose = Rover4wd_GetPose();
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    return pose;
}

Cavebot_Error_t Cavebot_SetWaypoint(const Cavebot_Pose_t *const waypoint)
{
    if 
}

Cavebot_Error_t Cavebot_RelativeMove(const Bsp_Meter_t position, const Bsp_Radian_t pose)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    BSP_UNUSED(pose);

    if (!Cavebot_Armed)
    {
        error = CAVEBOT_ERROR_MODE;
    }
    else if (Cavebot_RelativeMovingPosition || Cavebot_RelativeMovingPose)
    {
        error = CAVEBOT_ERROR_MOVE;
    }
    else if (position > 0.0)
    {
        /* TODO make extensible */
        Bsp_EncoderPulse_t pulses = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses + BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses + BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses + BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses;
        pulses = pulses / 4;

        Cavebot_RelativeMovePosition   = (kCavebot_MetersPerTick * pulses) + position;
        Cavebot_RelativeMovingPosition = true;
    }
    // else if (fabs(pose) > 0.0)
    // {
    //     // CavebotUser_Gyroscope.quaternion.y
    //     /* yaw=atan2(2(wz+xy),1−2(y^2+z^2)) */
    //     Bsp_Radian_t yaw = atan2(2.0 * ((CavebotUser_Gyroscope.quaternion.w * CavebotUser_Gyroscope.quaternion.z) + (CavebotUser_Gyroscope.quaternion.x * CavebotUser_Gyroscope.quaternion.y)),
    //                              1 - (2 * ((CavebotUser_Gyroscope.quaternion.y * CavebotUser_Gyroscope.quaternion.y) + (CavebotUser_Gyroscope.quaternion.z * CavebotUser_Gyroscope.quaternion.z))));
    //     Bsp_Radian_t delta_yaw = pose - yaw;
    //     delta_yaw                  = atan2(sin(delta_yaw), cos(delta_yaw));
    //     Cavebot_RelativeMovePose   = yaw + delta_yaw;
    //     Cavebot_RelativeMovingPose = true;
    // }

    return error;
}

bool Cavebot_IsRelativeMoving(void)
{
    return Cavebot_RelativeMovingPosition || Cavebot_RelativeMovingPose;
}

static Cavebot_Error_t Cavebot_Initialize(void)
{
    /* TODO CVW-21 read from config */
    Cavebot_Bot = CAVEBOT_BOT_4WD;

    Cavebot_Error_t error = CavebotUser_Initialize();

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Cavebot_Disarm();
    }

    return error;
}

static void Cavebot_Task(void)
{
    Cavebot_Error_t   error = CAVEBOT_ERROR_BOT;
    Bsp_Millisecond_t now   = BspTick_GetTick();

    /* TODO make extensible */
    /* TODO handle pose */
    if (((Cavebot_RelativeMoveTick - now) >= 10) && Cavebot_Armed)
    {
        if (Cavebot_RelativeMovingPosition)
        {
            Bsp_EncoderPulse_t pulses = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses + BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses + BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses + BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses;
            pulses = pulses / 4;
            Bsp_MetersPerSecond_t speed = CavebotMotionProfile_TrapezoidVelocity((kCavebot_MetersPerTick * pulses), Cavebot_RelativeMovePosition, 0.03, 0.64, 0.32); /* TODO CVW-21 read static params from config */
            Cavebot_Drive(speed, 0);
            Cavebot_RelativeMoveTick = now;

            if (speed <= 0.0)
            {
                Cavebot_RelativeMovingPosition = false;
            }
        }
        else if (Cavebot_RelativeMovingPose)
        {
            Rover4wd_DisableSpeedControl();
            Bsp_Radian_t yaw = atan2(2.0 * ((CavebotUser_Gyroscope.quaternion.w * CavebotUser_Gyroscope.quaternion.z) + (CavebotUser_Gyroscope.quaternion.x * CavebotUser_Gyroscope.quaternion.y)),
                                     1 - (2 * ((CavebotUser_Gyroscope.quaternion.y * CavebotUser_Gyroscope.quaternion.y) + (CavebotUser_Gyroscope.quaternion.z * CavebotUser_Gyroscope.quaternion.z))));
            /*             if (yaw >= Cavebot_RelativeMovePose)
                        {
                            yaw - Cavebot_RelativeMovePose;
                        } */
            // Bsp_RadiansPerSecond_t angular_speed = CavebotMotionProfile_TrapezoidVelocity(yaw, Cavebot_RelativeMovePose, 15, 20, 1);
            Bsp_Radian_t           error         = yaw - Cavebot_RelativeMovePose;
            Bsp_RadiansPerSecond_t angular_speed = 0.0;
            /* TODO */
            UNUSED(error);
            Cavebot_Drive(0, angular_speed);
            Cavebot_RelativeMoveTick = now;

            if (fabs(angular_speed) <= 0.0)
            {
                Cavebot_RelativeMovingPose = false;
                Rover4wd_EnableSpeedControl();
            }
        }
    }

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        error = Rover4ws_Task();
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        error = Rover4wd_Task();
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    if (CAVEBOT_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Task error %d", (int)error);
    }
}

static void Cavebot_MeasureLoopRate(void)
{
    static size_t            loop_count    = 0U;
    static Bsp_Microsecond_t previous_time = 0U;

    loop_count++;

    Bsp_Microsecond_t time       = BspTick_GetMicroseconds();
    Bsp_Microsecond_t difference = time - previous_time;
    if (difference >= CAVEBOT_LOOP_LOG_PERIOD)
    {
        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Loop rate %lfHz", (double)((double)loop_count / ((double)difference / BSP_TICK_MICROSECONDS_PER_SECOND)));
        loop_count    = 0;
        previous_time = time;
    }
}