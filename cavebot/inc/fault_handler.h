#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef uint32_t FaultHandler_Error_t;

typedef enum
{
    FAULT_HANDLER_FAULT_MEMORY,
    FAULT_HANDLER_FAULT_SCHEDULER,
    FAULT_HANDLER_FAULT_TIMER,
    FAULT_HANDLER_FAULT_MOTOR,
    FAULT_HANDLER_FAULT_ACCELEROMETER,
    FAULT_HANDLER_FAULT_GYROSCOPE,
    FAULT_HANDLER_FAULT_ENCODER,
    FAULT_HANDLER_FAULT_COMMS,
    FAULT_HANDLER_FAULT_LOGGING,
    FAULT_HANDLER_FAULT_RGBW,
    FAULT_HANDLER_FAULT_BUZZER,
    FAULT_HANDLER_FAULT_MAX
} FaultHandler_Fault_t;

/* TODO CVW-50 save file and line */
typedef struct
{
    const uint32_t threshold;
    volatile uint32_t count;
    volatile Bsp_Millisecond_t tick;
    volatile FaultHandler_Error_t error;
    volatile int line;
    volatile const char *file;
} FaultHandler_FaultState_t;

void FaultHandler_SetFault(const FaultHandler_Fault_t fault, const FaultHandler_Error_t error);
void FaultHandler_ClearFault(const FaultHandler_Fault_t fault);
bool FaultHandler_HasCriticalFaults(void);
bool FaultHandler_HasFault(const FaultHandler_Fault_t fault);

#endif /* FAULT_HANDLER_H */