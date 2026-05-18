#ifndef COMMS_H
#define COMMS_H

#include "cavebot.h"

Cavebot_Error_t Comms_Initialize(void);
void Comms_Task(void);

#endif /* COMMS_H */