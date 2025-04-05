#ifndef ROVER_PID_H
#define ROVER_PID_H

#include <stdbool.h>

#include "rover.h"

typedef struct
{
    double kp;
    double ki;
    double kd;
    double integral;
    double command;
    double error;
    double output;
    Rover_Microsecond_t previous_tick;
    bool enabled; /* TODO SD-125 feedback enabled */
    double minimum;
    double maximum;
} RoverPid_Handle_t;

Rover_Error_t RoverPid_Reset(RoverPid_Handle_t *const handle);
Rover_Error_t RoverPid_Enable(RoverPid_Handle_t *const handle);
Rover_Error_t RoverPid_Disable(RoverPid_Handle_t *const handle);
Rover_Error_t RoverPid_Update(RoverPid_Handle_t *const handle, const double actual, const Rover_Microsecond_t tick);

#endif /* ROVER_PID_H */