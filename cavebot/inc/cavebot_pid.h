#ifndef CAVEBOT_PID_H
#define CAVEBOT_PID_H

#include <stdbool.h>

#include "bsp.h"

#include "cavebot.h"

typedef struct
{
    double kp;
    double ki;
    double kd;
    double kff;
    double integral;
    double command;
    double error;
    double output;
    Bsp_Microsecond_t previous_tick;
    bool enabled; /* TODO SD-125 feedback enabled */
    double minimum;
    double maximum;
} CavebotPid_Handle_t;

Cavebot_Error_t CavebotPid_Reset(CavebotPid_Handle_t *const handle);
Cavebot_Error_t CavebotPid_Enable(CavebotPid_Handle_t *const handle);
Cavebot_Error_t CavebotPid_Disable(CavebotPid_Handle_t *const handle);
Cavebot_Error_t CavebotPid_Update(CavebotPid_Handle_t *const handle, const double actual, const Bsp_Microsecond_t tick);

#endif /* CAVEBOT_PID_H */