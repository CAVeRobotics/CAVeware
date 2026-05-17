#include "cavebot.h"

#include <math.h>
#include <stdbool.h>

#include "bsp.h"
#include "bsp_tick.h"
#include "bsp_logger.h"

#include "accelerometer.h"
#include "gyroscope.h"

#include "cavebot_user.h"
#include "comms.h"
#include "motion.h"
#include "scheduler.h"
#include "version.h"
#ifdef ROVER_4WD
#include "rover_4wd.h"
#endif
#ifdef ROVER_4WS
#include "rover_4ws.h"
#endif

#define CAVEBOT_LOOP_LOG_PERIOD (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)

static const char *      kCavebot_LogTag  = "CAVEBOT";
static Cavebot_Bot_t     Cavebot_Bot      = CAVEBOT_BOT_4WD;
static Cavebot_Mode_t    Cavebot_Mode     = CAVEBOT_MODE_DISARMED;
static Cavebot_Pose_t    Cavebot_Waypoint = {
    .x       = 0.0,
    .y       = 0.0,
    .heading = 0.0
};
static bool              Cavebot_HasWaypoint      = false; /* TODO replace with queue */
static Bsp_Microsecond_t Cavebot_WaypointTick     = 0U;
static Bsp_Meter_t       Cavebot_WaypointDistance = 0.0;

/* TODO CVW-21 read from config */
static const Bsp_Meter_t kCavebot_PositionTolerance = 0.05;

static Cavebot_Error_t Cavebot_Initialize(void);
static void Cavebot_MeasureLoopRate(void);
static void Cavebot_UpdateVelocity(void);

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
    else if (CAVEBOT_ERROR_NONE != Comms_Initialize())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start CAVeTalk");
    }
    else if (CAVEBOT_ERROR_NONE != Scheduler_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start scheduler");
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Initialized");

        while (true)
        {
            Scheduler_Run();
            Cavebot_MeasureLoopRate(); /* TODO CVW-71 move loop rate/task logging to scheduler */
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
        /* TODO set manual vs auto */
        // Cavebot_Mode = CAVEBOT_MODE_ARMED_MANUAL;
        Cavebot_Mode = CAVEBOT_MODE_ARMED_AUTO;

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
        Cavebot_Mode = CAVEBOT_MODE_DISARMED;

        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Disarmed", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to disarm with error %d", (int)error);
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
        error        = Cavebot_Drive(0.0, 0.0);
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

Cavebot_Error_t Cavebot_SetPose(const Cavebot_Pose_t *const pose)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_BOT;

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        /* TODO */
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        error = Rover4wd_SetPose(pose);
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    return error;
}

Bsp_MetersPerSecond_t Cavebot_GetLinearVelocity(void)
{
    Bsp_MetersPerSecond_t linear_velocity = 0.0;

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        /* TODO */
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        linear_velocity = Rover4wd_GetLinearVelocity();
#endif /* ROVER_4WD */
        break;
    default:
        break;
    }

    return linear_velocity;
}

Cavebot_Error_t Cavebot_SetWaypoint(const Cavebot_Pose_t *const waypoint)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;

    if (NULL == waypoint)
    {
        error = CAVEBOT_ERROR_NULL;
    }
    /* TODO */
    // else if (Cavebot_HasWaypoint)
    // {
    //     error = CAVEBOT_ERROR_MOVE;
    // }
    else
    {
        Cavebot_Waypoint     = *waypoint;
        Cavebot_WaypointTick = BspTick_GetMicroseconds();
        Cavebot_HasWaypoint  = true;
    }

    return error;
}

void Cavebot_Task(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_MODE;

    switch (Cavebot_Mode)
    {
    case CAVEBOT_MODE_ARMED_AUTO:
        Cavebot_UpdateVelocity();
        break;
    default:
        break;
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
        error = CAVEBOT_ERROR_BOT;
        break;
    }

    if (CAVEBOT_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Task error %d", (int)error);
    }
}

static Cavebot_Error_t Cavebot_Initialize(void)
{
    /* TODO CVW-21 read from config */
    Cavebot_Bot = CAVEBOT_BOT_4WD;

    Cavebot_Error_t error = Scheduler_Initialize();

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = CavebotUser_Initialize();
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Cavebot_Disarm();
    }

    return error;
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

static void Cavebot_UpdateVelocity(void)
{
    if (Cavebot_HasWaypoint)
    {
        Bsp_Microsecond_t  tick       = BspTick_GetMicroseconds();
        const Bsp_Second_t delta_time = BspTick_GetElapsedMicroseconds(Cavebot_WaypointTick, tick);
        Cavebot_WaypointTick = tick;

        /* Convert to local frame */
        const Cavebot_Pose_t pose        = Cavebot_GetPose();
        const Bsp_Meter_t    delta_x     = Cavebot_Waypoint.x - pose.x;
        const Bsp_Meter_t    delta_y     = Cavebot_Waypoint.y - pose.y;
        const Bsp_Radian_t   cos_heading = cos(pose.heading);
        const Bsp_Radian_t   sin_heading = sin(pose.heading);
        const Bsp_Meter_t    local_x     = (delta_x * cos_heading) - (delta_y * sin_heading);
        const Bsp_Meter_t    local_y     = (delta_x * sin_heading) + (delta_y * cos_heading);
        const Bsp_Meter_t    distance    = sqrt((local_x * local_x) + (local_y * local_y));
        const Bsp_Radian_t   angle       = atan2(local_y, local_x);

        if (Cavebot_WaypointDistance <= 0.0)
        {
            Cavebot_WaypointDistance = distance;
        }

        if (distance < kCavebot_PositionTolerance)
        {
            Cavebot_Drive(0.0, 0.0);
            Cavebot_HasWaypoint      = false;
            Cavebot_WaypointDistance = 0.0;
        }
        else
        {
            const Cavebot_Trajectory_t trajectory = Motion_Trapezoidal(Cavebot_WaypointDistance, distance, angle, delta_time);
            (void)Cavebot_Drive(trajectory.linear_velocity, trajectory.angular_velocity);
        }
    }
}