#include "fault_handler.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_tick.h"

#define FAULT_HANDLER_ERROR_NONE (FaultHandler_Error_t)0U

static volatile uint32_t                  FaultHandler_Mask                            = 0x00000000U;
static volatile FaultHandler_FaultState_t FaultHandler_Faults[FAULT_HANDLER_FAULT_MAX] = {
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
    /* TODO CVW-50 */

    return false;
}

bool FaultHandler_HasFault(const FaultHandler_Fault_t fault)
{
    bool has_fault = false;

    if ((fault < FAULT_HANDLER_FAULT_MAX) && ((FaultHandler_Mask & (1U << fault)) > 0U))
    {
        has_fault = true;
    }

    return has_fault;
}