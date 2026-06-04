#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>

#define COMMS_CAVETALK_ID 0x00000001U

bool Comms_Initialize(void);
void Comms_Task(void);

#endif /* COMMS_H */