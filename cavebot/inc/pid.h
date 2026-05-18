#ifndef PID_H
#define PID_H

#include <stdbool.h>

#include "bsp.h"

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
} Pid_Handle_t;

void Pid_Reset(Pid_Handle_t *const handle);
void Pid_Enable(Pid_Handle_t *const handle);
void Pid_Disable(Pid_Handle_t *const handle);
void Pid_Update(Pid_Handle_t *const handle, const double actual);

#endif /* PID_H */