#include "faults.h"

#include <stdint.h>

#define FAULTS_ERROR_NONE (Fault_Error_t)0U

static uint32_t Faults_Mask       = 0x00000000U;
static Fault_t  Faults[FAULT_MAX] = {
    0 /* TODO */
};

void Fault_SetFault(const Fault_t fault, const Fault_Error_t error)
{
    BSP_UNUSED(fault);
    BSP_UNUSED(error);

    /* TODO */
}
