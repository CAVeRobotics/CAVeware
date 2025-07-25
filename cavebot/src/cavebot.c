#include "cavebot.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"
#include "bsp_tick.h"

#include "cavebot_buttons.h"
#include "cavebot_cavetalk.h"
#include "cavebot_version.h"
#include "rover.h"
#include "rover_4ws.h"

#define CAVEBOT_LOOP_LOG_PERIOD (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)

static const char *kCavebot_LogTag = "CAVEMAN";

static void Cavebot_Initialize(void);
static void Cavebot_MeasureLoopRate(void);

int main(void)
{
    Cavebot_Initialize();

    while (true)
    {
        Rover_Task();
        CavebotCaveTalk_Task();
        Cavebot_MeasureLoopRate();
    }

    return 0;
}

static void Cavebot_Initialize(void)
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

    Rover_Initialize();

    if (CAVE_TALK_ERROR_NONE != CavebotCaveTalk_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to start CAVeTalk");
    }

    if (BSP_ERROR_NONE != CavebotButtons_Enable(CAVEBOT_BUTTONS_BUTTON_START))
    {
        BSP_LOGGER_LOG_ERROR(kCavebot_LogTag, "Failed to enabled buttons");
    }

    /* TODO SD-348 testing */
    (void)Rover4ws_DisableSpeedControl();
    (void)Rover4ws_DisableSteeringControl();

    BSP_LOGGER_LOG_INFO(kCavebot_LogTag, "Initialized");
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