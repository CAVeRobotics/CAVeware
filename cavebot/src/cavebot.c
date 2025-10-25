#include "cavebot.h"

#include <stdbool.h>

#include "bsp.h"
#include "bsp_tick.h"
#include "bsp_logger.h"

#include "accelerometer.h"
#include "gyroscope.h"

#ifdef CAVEBOARD_MINI
#include "a4988.h"
#endif /* CAVEBOARD_MINI */

#include "cavebot_cavetalk.h"
#include "cavebot_user.h"
#include "cavebot_version.h"
#ifdef ROVER_4WD
#include "rover_4wd.h"
#endif
#ifdef ROVER_4WS
#include "rover_4ws.h"
#endif

#define CAVEBOT_LOOP_LOG_PERIOD (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)

static const char *  kCavebot_LogTag = "CAVEBOT";
static bool          Cavebot_Armed   = false;
static Cavebot_Bot_t Cavebot_Bot     = CAVEBOT_BOT_4WD;

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

        Cavebot_Arm();
        Cavebot_Drive(0, 3);

        while (true)
        {
            Cavebot_Task();
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
    return Cavebot_Armed;
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

static Cavebot_Error_t Cavebot_Initialize(void)
{
    /* TODO move to CavebotUser_Initialize */
    (void)Accelerometer_Initialize(&CavebotUser_Accelerometer);
    (void)Gyroscope_Initialize(&CavebotUser_Gyroscope);

    /* TODO CVW-21 read from config */
    Cavebot_Bot = CAVEBOT_BOT_4WD;

    Cavebot_Error_t error = Cavebot_Disarm();

    return error;
}

static void Cavebot_Task(void)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_BOT;

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