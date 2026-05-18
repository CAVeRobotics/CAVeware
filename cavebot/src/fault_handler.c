#include "fault_handler.h"

#include <stdbool.h>
#include <stdint.h>

#define FAULT_HANDLER_ERROR_NONE (FaultHandler_Error_t)0U

static uint32_t             FaultHandler_Mask                            = 0x00000000U;
static FaultHandler_Fault_t FaultHandler_Faults[FAULT_HANDLER_FAULT_MAX] = {
    0 /* TODO CVW-50 */
};

void FaultHandler_SetFault(const FaultHandler_Fault_t fault, const FaultHandler_Error_t error)
{
    BSP_UNUSED(fault);
    BSP_UNUSED(error);

    /* TODO CVW-50 */
    /* TODO CVW-50 adding debug logging */
}

void FaultHandler_ClearFault(const FaultHandler_Fault_t fault)
{
    BSP_UNUSED(fault);

    /* TODO CVW-50 */
}

bool FaultHandler_HasCriticalFaults(void)
{
    /* TODO CVW-50 */

    return false;
}