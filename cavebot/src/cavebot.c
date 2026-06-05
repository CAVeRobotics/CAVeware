#include "cavebot.h"

#include <math.h>
#include <stdbool.h>

#include "cavetalk.h"

#include "bsp.h"
#include "bsp_tick.h"
#include "bsp_logger.h"

#include "cavebot_user.h"
#include "comms.h"
#include "fault_handler.h"
#include "fsm.h"
#include "motion.h"
#include "scheduler.h"
#include "version.h"
#ifdef ROVER_4WD
#include "rover_4wd.h"
#endif
#ifdef ROVER_4WS
#include "rover_4ws.h"
#endif

#define CAVEBOT_LOOP_LOG_PERIOD   (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)
#define CAVEBOT_COMMS_TASK_PERIOD (Scheduler_Tick_t)1U
#define CAVEBOT_TASK_PERIOD       (Scheduler_Tick_t)4U

static const char *  kCavebot_LogTag = "CAVEBOT";
static Cavebot_Bot_t Cavebot_Bot     = CAVEBOT_BOT_4WD; /* TODO CVW-21 read from config */

static Cavebot_Pose_t    Cavebot_Waypoint = {
    .x       = 0.0,
    .y       = 0.0,
    .heading = 0.0
};
static bool              Cavebot_HasWaypoint        = false; /* TODO replace with queue */
static Bsp_Microsecond_t Cavebot_WaypointTick       = 0U;
static Bsp_Meter_t       Cavebot_WaypointDistance   = 0.0;
static const Bsp_Meter_t kCavebot_PositionTolerance = 0.05; /* TODO CVW-21 read from config */

static void Cavebot_EnterInitialize(void);
static Fsm_State_t *Cavebot_UpdateInitialize(void);
static void Cavebot_EnterReady(void);
static Fsm_State_t *Cavebot_UpdateReady(void);
static void Cavebot_EnterManual(void);
static Fsm_State_t *Cavebot_UpdateManual(void);
static void Cavebot_EnterAuto(void);
static Fsm_State_t *Cavebot_UpdateAuto(void);
static void Cavebot_EnterFailed(void);
static Fsm_State_t *Cavebot_UpdateFailed(void);
static void Cavebot_Arm(void);
static void Cavebot_Disarm(void);
static void Cavebot_Run(void);
static void Cavebot_Task(void);
static void Cavebot_MeasureLoopRate(void);
static void Cavebot_UpdateVelocity(void);

static Fsm_t             Cavebot_Fsm;
static const char *const Cavebot_FsmName                   = "CAVEBOT FSM";
static Cavebot_State_t   Cavebot_State                     = CAVEBOT_STATE_INITIALIZE;
static Cavebot_State_t   Cavebot_RequestedState            = CAVEBOT_STATE_INITIALIZE;
static Fsm_State_t       Cavebot_States[CAVEBOT_STATE_MAX] = {
    [CAVEBOT_STATE_INITIALIZE] = {
        .name   = "INITIALIZE",
        .enter  = Cavebot_EnterInitialize,
        .update = Cavebot_UpdateInitialize,
        .exit   = NULL,
    },
    [CAVEBOT_STATE_READY] = {
        .name   = "READY",
        .enter  = Cavebot_EnterReady,
        .update = Cavebot_UpdateReady,
        .exit   = NULL,
    },
    [CAVEBOT_STATE_MANUAL] = {
        .name   = "MANUAL",
        .enter  = Cavebot_EnterManual,
        .update = Cavebot_UpdateManual,
        .exit   = NULL,
    },
    [CAVEBOT_STATE_AUTO] = {
        .name   = "AUTO",
        .enter  = Cavebot_EnterAuto,
        .update = Cavebot_UpdateAuto,
        .exit   = NULL,
    },
    [CAVEBOT_STATE_FAILED] = {
        .name   = "FAILED",
        .enter  = Cavebot_EnterFailed,
        .update = Cavebot_UpdateFailed,
        .exit   = NULL,
    },
};

int main(void)
{
    Bsp_Initialize(); /* TODO create custom logger in cavebot_user to set logging faults */

    /* Immediately print out build info in case there is a problem starting BSP tick */
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build branch: %s", CAVEBOT_GIT_BRANCH);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build commit: %s (%s)", CAVEBOT_GIT_COMMIT_HASH, CAVEBOT_GIT_DIRTY);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build tag: %s", CAVEBOT_GIT_TAG);

    Bsp_Error_t error = BspTick_Start();
    if (BSP_ERROR_NONE != error)
    {
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_TIMER, error);
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start BSP Tick");
    }
    else if (!Scheduler_Initialize())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to initialize scheduler");
    }
    else if (!Fsm_Initialize(&Cavebot_Fsm, &Cavebot_States[CAVEBOT_STATE_INITIALIZE], Cavebot_FsmName))
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to initialize FSM");
    }
    else if (!Comms_Initialize())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to initialize comms");
    }
    else
    {
        if (!CavebotUser_Initialize())
        {
            BSP_LOGGER_LOG_WARNING(kCavebot_LogTag, "Failed to initialize board");
        }

        if (!Scheduler_AddTask(Comms_Task, CAVEBOT_COMMS_TASK_PERIOD) || Scheduler_AddTask(Cavebot_Task, CAVEBOT_TASK_PERIOD))
        {
            BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to add tasks to scheduler");
        }
        else if (!Scheduler_Start())
        {
            BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start scheduler");
        }
        else
        {
            BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Initialization finished");

            while (true)
            {
                Scheduler_Run();
                Cavebot_MeasureLoopRate(); /* TODO CVW-71 move loop rate/task logging to scheduler */
            }
        }
    }

    return 0;
}

Cavebot_State_t Cavebot_GetState(void)
{
    return Cavebot_State;
}

bool Cavebot_SetState(const Cavebot_State_t state)
{
    Cavebot_RequestedState = Cavebot_State;

    switch (Cavebot_State)
    {
    case CAVEBOT_STATE_READY:
        if ((CAVEBOT_STATE_MANUAL == state) || (CAVEBOT_STATE_AUTO == state))
        {
            Cavebot_RequestedState = state;
        }
        break;
    case CAVEBOT_STATE_MANUAL:
        if ((CAVEBOT_STATE_READY == state) || (CAVEBOT_STATE_AUTO == state))
        {
            Cavebot_RequestedState = state;
        }
        break;
    case CAVEBOT_STATE_AUTO:
        if ((CAVEBOT_STATE_READY == state) || (CAVEBOT_STATE_MANUAL == state))
        {
            Cavebot_RequestedState = state;
        }
        break;
    case CAVEBOT_STATE_INITIALIZE:
    case CAVEBOT_STATE_FAILED:
    case CAVEBOT_STATE_MAX:
    default:
        break;
    }

    return Cavebot_RequestedState != Cavebot_State;
}

bool Cavebot_IsArmed(void)
{
    bool armed = false;

    switch (Cavebot_State)
    {
    case CAVEBOT_STATE_MANUAL:
    case CAVEBOT_STATE_AUTO:
        armed = true;
        break;
    case CAVEBOT_STATE_INITIALIZE:
    case CAVEBOT_STATE_READY:
    case CAVEBOT_STATE_FAILED:
    case CAVEBOT_STATE_MAX:
    default:
        break;
    }

    return armed;
}

void Cavebot_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        Rover4ws_Drive(speed, turn_rate);
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        Rover4wd_Drive(speed, turn_rate);
#endif /* ROVER_4WD */
        break;
    default:
        /* TODO handle invalid bot */
        break;
    }
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
        /* TODO handle invalid bot */
        break;
    }

    return pose;
}

void Cavebot_SetPose(const Cavebot_Pose_t *const pose)
{
    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        /* TODO */
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        Rover4wd_SetPose(pose);
#endif /* ROVER_4WD */
        break;
    default:
        /* TODO handle invalid bot */
        break;
    }
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
        /* TODO handle invalid bot */
        break;
    }

    return linear_velocity;
}

bool Cavebot_SetWaypoint(const Cavebot_Pose_t *const waypoint)
{
    bool waypoint_set = false;

    if (NULL == waypoint)
    {
        /* Do nothing */
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
        waypoint_set         = true;
    }

    return waypoint_set;
}

static void Cavebot_EnterInitialize(void)
{
    Cavebot_State = CAVEBOT_STATE_INITIALIZE;
}

static Fsm_State_t *Cavebot_UpdateInitialize(void)
{
    Fsm_State_t *next = &Cavebot_States[CAVEBOT_STATE_READY];

    if (FaultHandler_HasCriticalFaults())
    {
        next = &Cavebot_States[CAVEBOT_STATE_FAILED];
    }

    return next;
}

static void Cavebot_EnterReady(void)
{
    Cavebot_State = CAVEBOT_STATE_READY;

    Cavebot_Disarm();
    Comms_SpeakGetMode(cavetalk_Mode_MODE_DISARMED);
}

static Fsm_State_t *Cavebot_UpdateReady(void)
{
    Fsm_State_t *next = &Cavebot_States[CAVEBOT_STATE_READY];

    if (FaultHandler_HasCriticalFaults())
    {
        next = &Cavebot_States[CAVEBOT_STATE_FAILED];
    }
    else if ((CAVEBOT_STATE_MANUAL == Cavebot_RequestedState) || (CAVEBOT_STATE_AUTO == Cavebot_RequestedState))
    {
        next = &Cavebot_States[Cavebot_RequestedState];
    }

    return next;
}

static void Cavebot_EnterManual(void)
{
    Cavebot_State = CAVEBOT_STATE_MANUAL;

    Cavebot_Arm();
    Comms_SpeakGetMode(cavetalk_Mode_MODE_ARMED_MANUAL);
}

static Fsm_State_t *Cavebot_UpdateManual(void)
{
    Fsm_State_t *next = &Cavebot_States[CAVEBOT_STATE_MANUAL];

    if (FaultHandler_HasCriticalFaults())
    {
        next = &Cavebot_States[CAVEBOT_STATE_FAILED];
    }
    else if ((CAVEBOT_STATE_READY == Cavebot_RequestedState) || (CAVEBOT_STATE_AUTO == Cavebot_RequestedState))
    {
        next = &Cavebot_States[Cavebot_RequestedState];
    }
    else
    {
        Cavebot_Run();
    }

    return next;
}

static void Cavebot_EnterAuto(void)
{
    Cavebot_State = CAVEBOT_STATE_AUTO;

    Cavebot_Arm();
    Comms_SpeakGetMode(cavetalk_Mode_MODE_ARMED_AUTO);
}

static Fsm_State_t *Cavebot_UpdateAuto(void)
{
    Fsm_State_t *next = &Cavebot_States[CAVEBOT_STATE_AUTO];

    if (FaultHandler_HasCriticalFaults())
    {
        next = &Cavebot_States[CAVEBOT_STATE_FAILED];
    }
    else if ((CAVEBOT_STATE_READY == Cavebot_RequestedState) || (CAVEBOT_STATE_MANUAL == Cavebot_RequestedState))
    {
        next = &Cavebot_States[Cavebot_RequestedState];
    }
    else
    {
        Cavebot_UpdateVelocity();
        Cavebot_Run();
    }

    return next;
}

static void Cavebot_EnterFailed(void)
{
    Cavebot_State = CAVEBOT_STATE_FAILED;

    Cavebot_Disarm();
}

static Fsm_State_t *Cavebot_UpdateFailed(void)
{
    Fsm_State_t *next = &Cavebot_States[CAVEBOT_STATE_FAILED];

    if (!FaultHandler_HasCriticalFaults())
    {
        next = &Cavebot_States[CAVEBOT_STATE_READY];
    }

    return next;
}

static void Cavebot_Arm(void)
{
    /* TODO add logging in bots */

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        (void)Rover4ws_Arm();
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        (void)Rover4wd_Arm();
#endif /* ROVER_4WD */
        break;
    default:
        /* TODO handle invalid bot */
        break;
    }
}

static void Cavebot_Disarm(void)
{
    /* TODO add logging in bots */

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        (void)Rover4ws_Disarm();
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        (void)Rover4wd_Disarm();
#endif /* ROVER_4WD */
        break;
    default:
        /* TODO handle invalid bot */
        break;
    }
}

static void Cavebot_Run(void)
{
    /* TODO add logging in bots */

    switch (Cavebot_Bot)
    {
    case CAVEBOT_BOT_4WS:
#ifdef ROVER_4WS
        (void)Rover4ws_Run();
#endif /* ROVER_4WS */
        break;
    case CAVEBOT_BOT_4WD:
#ifdef ROVER_4WD
        (void)Rover4wd_Run();
#endif /* ROVER_4WD */
        break;
    default:
        /* TODO handle invalid bot */
        break;
    }
}

static void Cavebot_Task(void)
{
    Fsm_Update(&Cavebot_Fsm);
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