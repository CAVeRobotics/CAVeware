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
    double rate_limit;
    double integral;
    double command;
    double error;
    double output;
    Bsp_Microsecond_t previous_tick;
    bool enabled;
    bool integral_enabled;
    double minimum;
    double maximum;
} CavebotPid_Handle_t;

Cavebot_Error_t CavebotPid_Reset(CavebotPid_Handle_t *const handle);
Cavebot_Error_t CavebotPid_Enable(CavebotPid_Handle_t *const handle);
Cavebot_Error_t CavebotPid_Disable(CavebotPid_Handle_t *const handle);
Cavebot_Error_t CavebotPid_Update(CavebotPid_Handle_t *const handle, const double actual);

#endif /* CAVEBOT_PID_H */