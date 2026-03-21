#include "cavebot_scheduler.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_timer.h"
#include "bsp_timer_user.h"

#include "cavebot.h"

#define CAVEBOT_SCHEDULER_TIMER     BSP_TIMER_USER_TIMER_0
#define CAVEBOT_SCHEDULER_MAX_TASKS 16U

/* TODO CVW-71 add task priorities */
typedef struct
{
    void (*task)(void);
    CavebotScheduler_Tick_t interval;
    volatile CavebotScheduler_Tick_t counter;
    volatile size_t overrun_counter;
    volatile bool due;
} CavebotScheduler_Task_t;

static CavebotScheduler_Task_t CavebotScheduler_Tasks[CAVEBOT_SCHEDULER_MAX_TASKS] = {
    0
};
static size_t                  CavebotScheduler_TaskCount = 0U;

static void CavebotScheduler_Task(void *arg);

Cavebot_Error_t CavebotScheduler_Initialize(void)
{
    Bsp_Callback_t callback = {
        .function = CavebotScheduler_Task,
        .arg      = NULL
    };

    return Cavebot_BspToCavebotError(BspTimer_RegisterPeriodElapsedCallback(CAVEBOT_SCHEDULER_TIMER, &callback));
}

Cavebot_Error_t CavebotScheduler_AddTask(void (*task)(void), const CavebotScheduler_Tick_t interval)
{
    Cavebot_Error_t error = CAVEBOT_ERROR_NULL;

    if (NULL != task)
    {
        CavebotScheduler_Tasks[CavebotScheduler_TaskCount] = (CavebotScheduler_Task_t){
            .task            = task,
            .interval        = interval,
            .counter         = 0U,
            .overrun_counter = 0U,
            .due             = false,
        };
        CavebotScheduler_TaskCount++;
    }

    return error;
}

Cavebot_Error_t CavebotScheduler_Start(void)
{
    return Cavebot_BspToCavebotError(BspTimer_Start(CAVEBOT_SCHEDULER_TIMER));
}

Cavebot_Error_t CavebotScheduler_Stop(void)
{
    return Cavebot_BspToCavebotError(BspTimer_Stop(CAVEBOT_SCHEDULER_TIMER));
}

void CavebotScheduler_Run(void)
{
    for (size_t task = 0U; task < CavebotScheduler_TaskCount; task++)
    {
        /* TODO CVW-71 log task overrun warning */

        if (CavebotScheduler_Tasks[task].due)
        {
            CavebotScheduler_Tasks[task].due = false;

            CavebotScheduler_Tasks[task].task();
        }
    }
}

static void CavebotScheduler_Task(void *arg)
{
    BSP_UNUSED(arg);

    for (size_t task = 0U; task < CavebotScheduler_TaskCount; task++)
    {
        CavebotScheduler_Tasks[task].counter++;

        if (CavebotScheduler_Tasks[task].counter >= CavebotScheduler_Tasks[task].interval)
        {
            if (CavebotScheduler_Tasks[task].due)
            {
                CavebotScheduler_Tasks[task].overrun_counter++;
            }

            CavebotScheduler_Tasks[task].due     = true;
            CavebotScheduler_Tasks[task].counter = 0U;
        }
    }
}