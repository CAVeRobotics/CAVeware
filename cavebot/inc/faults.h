#ifndef FAULTS_H
#define FAULTS_H

#include <stdint.h>

#include "bsp.h"

typedef uint32_t Fault_Error_t;

typedef enum
{
    FAULT_MEMORY,
    FAULT_SCHEDULER,
    FAULT_ACCELEROMETER,
    FAULT_GYROSCOPE,
    FAULT_ENCODER,
    FAULT_COMMS,
    FAULT_LOGGING,
    FAULT_LED,
    FAULT_BUZZER,
    FAULT_MAX
} Fault_t;

typedef struct
{
    uint32_t threshold;
    uint32_t count;
    Bsp_Millisecond_t tick_first;
    Bsp_Millisecond_t tick_current;
    Fault_Error_t code_first;
    Fault_Error_t code_current;
} Fault_State_t;

void Fault_SetFault(const Fault_t fault, const Fault_Error_t error);

#endif /* FAULT_H */