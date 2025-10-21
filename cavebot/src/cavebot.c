#include "cavebot.h"

#include <stdbool.h>

#include "bsp.h"
#include "bsp_tick.h"
#include "bsp_logger.h"

#include "accelerometer.h"
#include "gyroscope.h"

#include "cavebot_cavetalk.h"
#include "cavebot_user.h"
#include "cavebot_version.h"
#ifdef BOARD_CAVEBOARD
#include "rover_4ws.h"
#endif

#define CAVEBOT_LOOP_LOG_PERIOD (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)

static const char *kCavebot_LogTag = "CAVEBOT";
static bool        Cavebot_Armed   = false;

static void Cavebot_Initialize(void);
static void Cavebot_Task(void);
static void Cavebot_MeasureLoopRate(void);

int main(void)
{
    Bsp_Initialize();

    if (BSP_ERROR_NONE != BspTick_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start BSP Tick");
    }

    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build branch: %s", CAVEBOT_GIT_BRANCH);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build commit: %s", CAVEBOT_GIT_COMMIT_HASH);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build tag: %s", CAVEBOT_GIT_TAG);
    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Build status: %s", CAVEBOT_GIT_DIRTY);

    Cavebot_Initialize();

    if (CAVE_TALK_ERROR_NONE != CavebotCaveTalk_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start CAVeTalk");
    }

    /* TODO SD-348 testing */
#ifdef BOARD_CAVEBOARD
    (void)Rover4ws_DisableSpeedControl();
    (void)Rover4ws_DisableSteeringControl();
#endif

    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Initialized");

    while (true)
    {
        Cavebot_Task();
        CavebotCaveTalk_Task();
        Cavebot_MeasureLoopRate();
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
#ifdef BOARD_CAVEBOARD
    Cavebot_Error_t error = Rover4ws_EnableSteering();
#else
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
#endif

#ifdef BOARD_CAVEBOARD
    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_StartMotors();
    }
#endif

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
#ifdef BOARD_CAVEBOARD
    Cavebot_Error_t error = Rover4ws_DisableSteering();
#else
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
#endif

#ifdef BOARD_CAVEBOARD
    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Rover4ws_StopMotors();
    }
#endif

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

Cavebot_Error_t Cavebot_EnableControl(void)
{
#ifdef BOARD_CAVEBOARD
    Cavebot_Error_t error = Rover4ws_EnableSpeedControl();
#else
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
#endif

    if (CAVEBOT_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to enable control with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Control enabled");
    }

    return error;
}

Cavebot_Error_t Cavebot_DisableControl(void)
{
#ifdef BOARD_CAVEBOARD
    Cavebot_Error_t error = Rover4ws_DisableSpeedControl();
#else
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
#endif

    if (CAVEBOT_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to disable control with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Control disabled");
    }

    return error;
}

Cavebot_Error_t Cavebot_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate)
{
#ifdef BOARD_CAVEBOARD
    Cavebot_Error_t error = Rover4ws_Drive(speed, turn_rate);
#else
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
#endif

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

static void Cavebot_Initialize(void)
{
    /* TODO CVW-50 handle return codes */
    (void)Cavebot_Disarm();
    (void)Accelerometer_Initialize(&CavebotUser_Accelerometer);
    (void)Gyroscope_Initialize(&CavebotUser_Gyroscope);
}

static void Cavebot_Task(void)
{
#ifdef BOARD_CAVEBOARD
    Cavebot_Error_t error = Rover4ws_Task();
#else
    Cavebot_Error_t error = CAVEBOT_ERROR_NONE;
#endif

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