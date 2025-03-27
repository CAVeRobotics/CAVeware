#include "caveman_controller.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"
#include "bsp_tick.h"

#include "caveman_buttons.h"
#include "caveman_cavetalk.h"
#include "rover.h"

#define CAVEMAN_LOOP_LOG_PERIOD (Bsp_Microsecond_t)((Bsp_Microsecond_t)5U * BSP_TICK_MICROSECONDS_PER_SECOND)

static const char *kCaveman_LogTag = "CAVEMAN";

static void Caveman_Initialize(void);
static void Caveman_MeasureLoopRate(void);

int main(void)
{
    Caveman_Initialize();

    while (true)
    {
        Rover_Task();
        CavemanCaveTalk_Task();
        Caveman_MeasureLoopRate();
    }

    return 0;
}

static void Caveman_Initialize(void)
{
    Bsp_Initialize();

    if (BSP_ERROR_NONE != BspTick_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCaveman_LogTag, "Failed to start BSP Tick");
    }

    Rover_Initialize();

    if (CAVE_TALK_ERROR_NONE != CavemanCaveTalk_Start())
    {
        BSP_LOGGER_LOG_ERROR(kCaveman_LogTag, "Failed to start CAVeTalk");
    }

    CavemanButtons_Enable(CAVEMAN_BUTTONS_BUTTON_HEADLIGHTS);
    CavemanButtons_Enable(CAVEMAN_BUTTONS_BUTTON_START);

    BSP_LOGGER_LOG_INFO(kCaveman_LogTag, "Initialized");
}

static void Caveman_MeasureLoopRate(void)
{
    static size_t            loop_count    = 0U;
    static Bsp_Microsecond_t previous_time = 0U;

    loop_count++;

    Bsp_Microsecond_t time       = BspTick_GetMicroseconds();
    Bsp_Microsecond_t difference = time - previous_time;
    if (difference >= CAVEMAN_LOOP_LOG_PERIOD)
    {
        BSP_LOGGER_LOG_INFO(kCaveman_LogTag, "Loop rate %lfHz", (double)((double)loop_count / ((double)difference / BSP_TICK_MICROSECONDS_PER_SECOND)));
        loop_count    = 0;
        previous_time = time;
    }
}