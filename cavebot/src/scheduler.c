#include "scheduler.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_timer.h"
#include "bsp_timer_user.h"

#include "faults.h"

#define CAVEBOT_SCHEDULER_TIMER     BSP_TIMER_USER_TIMER_0
#define CAVEBOT_SCHEDULER_MAX_TASKS 16U

/* TODO CVW-71 add task priorities */
typedef struct
{
    void (*task)(void);
    Scheduler_Tick_t interval;
    volatile Scheduler_Tick_t counter;
    volatile size_t overrun_counter;
    volatile bool due;
} Scheduler_Task_t;

static Scheduler_Task_t Scheduler_Tasks[CAVEBOT_SCHEDULER_MAX_TASKS] = {
    0
};
static size_t           Scheduler_TaskCount = 0U;

static void Scheduler_Task(void *arg);

bool Scheduler_Initialize(void)
{
    const Bsp_Callback_t callback = {
        .function = Scheduler_Task,
        .arg      = NULL
    };
    const Bsp_Error_t error = BspTimer_RegisterPeriodElapsedCallback(CAVEBOT_SCHEDULER_TIMER, &callback);
    bool initialized = true;

    if (BSP_ERROR_NONE != error)
    {
        Faults_SetFault(FAULT_SCHEDULER, error);
        initialized = false;
    }

    return initialized;
}

bool Scheduler_AddTask(void (*task)(void), const Scheduler_Tick_t interval)
{
    bool added = false;

    if ((NULL != task) && (Scheduler_TaskCount < CAVEBOT_SCHEDULER_MAX_TASKS))
    {
        Scheduler_Tasks[Scheduler_TaskCount] = (Scheduler_Task_t){
            .task            = task,
            .interval        = interval,
            .counter         = 0U,
            .overrun_counter = 0U,
            .due             = false,
        };
        Scheduler_TaskCount++;
        added = true;
    }

    return added;
}

bool Scheduler_Start(void)
{
    const Bsp_Error_t error = BspTimer_Start(CAVEBOT_SCHEDULER_TIMER);
    bool started = true;

    if (BSP_ERROR_NONE != error)
    {
        Faults_SetFault(FAULT_SCHEDULER, error);
        started = false;
    }

    return started;
}

bool Scheduler_Stop(void)
{
    const Bsp_Error_t error = BspTimer_Stop(CAVEBOT_SCHEDULER_TIMER);
    bool stopped = true;

    if (BSP_ERROR_NONE != error)
    {
        Faults_SetFault(FAULT_SCHEDULER, error);
        stopped = false;
    }

    return stopped;
}

void Scheduler_Run(void)
{
    for (size_t task = 0U; task < Scheduler_TaskCount; task++)
    {
        /* TODO CVW-71 log task overrun warning */

        if (Scheduler_Tasks[task].due)
        {
            Scheduler_Tasks[task].due = false;

            Scheduler_Tasks[task].task();
        }
    }
}

static void Scheduler_Task(void *arg)
{
    BSP_UNUSED(arg);

    for (size_t task = 0U; task < Scheduler_TaskCount; task++)
    {
        Scheduler_Tasks[task].counter++;

        if (Scheduler_Tasks[task].counter >= Scheduler_Tasks[task].interval)
        {
            if (Scheduler_Tasks[task].due)
            {
                Scheduler_Tasks[task].overrun_counter++;
            }

            Scheduler_Tasks[task].due     = true;
            Scheduler_Tasks[task].counter = 0U;
        }
    }
}