#include "fsm.h"

#include <stdbool.h>

#include "bsp_logger.h"

static const char *kFsmLogTag = "FSM";

bool Fsm_Initialize(Fsm_t *const fsm, Fsm_State_t *const state, const char *const name)
{
    bool initialized = false;

    if ((NULL != fsm) && (NULL != state) && (NULL != name))
    {
        fsm->name  = name;
        fsm->state = state;

        if (NULL != state->enter)
        {
            state->enter();
        }

        BSP_LOGGER_LOG_INFO(kFsmLogTag, "%s initialized to state %s", fsm->name, fsm->state->name);

        initialized = true;
    }

    return initialized;
}

void Fsm_Update(Fsm_t *const fsm)
{
    if (NULL != fsm)
    {
        Fsm_State_t *next = NULL;

        if (NULL != fsm->state->update)
        {
            next = fsm->state->update();
        }

        if ((NULL != next) && (next != fsm->state))
        {
            BSP_LOGGER_LOG_INFO(kFsmLogTag, "%s: %s -> %s", fsm->name, fsm->state->name, next->name);

            if (NULL != fsm->state->exit)
            {
                fsm->state->exit();
            }

            fsm->state = next;

            if (NULL != fsm->state->enter)
            {
                fsm->state->enter();
            }
        }
    }
}