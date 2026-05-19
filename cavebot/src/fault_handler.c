#include "fault_handler.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_tick.h"

#define FAULT_HANDLER_ERROR_NONE    (FaultHandler_Error_t)0U
#define FAULT_HANDLER_CRITICAL_MASK (uint32_t)0x7FU

static volatile uint32_t         FaultHandler_Mask                            = 0x00000000U;
static FaultHandler_FaultState_t FaultHandler_Faults[FAULT_HANDLER_FAULT_MAX] = {
    [FAULT_HANDLER_FAULT_MEMORY] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_SCHEDULER] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_TIMER] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_MOTOR] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_ACCELEROMETER] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_GYROSCOPE] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_ENCODER] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_COMMS] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_LOGGING] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_RGBW] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
    [FAULT_HANDLER_FAULT_BUZZER] = {
        .threshold = 1U,
        .count     = 0U,
        .tick      = 0U,
        .error     = FAULT_HANDLER_ERROR_NONE,
        .line      = 0,
        .file      = NULL,
    },
};

/* TODO CVW-50 make interrupt safe */
void FaultHandler_SetFault(const FaultHandler_Fault_t fault, const FaultHandler_Error_t error)
{
    if ((fault < FAULT_HANDLER_FAULT_MAX) && (error > FAULT_HANDLER_ERROR_NONE))
    {
        FaultHandler_FaultState_t *const state = &FaultHandler_Faults[fault];

        state->count++;

        if (state->count >= state->threshold)
        {
            FaultHandler_Mask |= (1U << fault);
            state->tick        = BspTick_GetTick();
            state->error       = error;

            /* TODO CVW-50 debug log with file and line number */
        }
    }
}

/* TODO CVW-50 make interrupt safe */
void FaultHandler_ClearFault(const FaultHandler_Fault_t fault)
{
    if (fault < FAULT_HANDLER_FAULT_MAX)
    {
        FaultHandler_Mask               &= ~(1U << fault);
        FaultHandler_Faults[fault].count = 0U;
    }
}

bool FaultHandler_HasCriticalFaults(void)
{
    const uint32_t mask = FaultHandler_Mask;

    return 0U != (mask & FAULT_HANDLER_CRITICAL_MASK);
}

bool FaultHandler_HasFault(const FaultHandler_Fault_t fault)
{
    bool           has_fault = false;
    const uint32_t mask      = FaultHandler_Mask;

    if ((fault < FAULT_HANDLER_FAULT_MAX) && (0U != (mask & (1U << fault))))
    {
        has_fault = true;
    }

    return has_fault;
}