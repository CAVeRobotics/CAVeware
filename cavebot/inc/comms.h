#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cavetalk.h"

#define COMMS_CAVETALK_ID 0x00000001U

bool Comms_Initialize(void);
void Comms_Task(void);
void Comms_SpeakLog(char *const log);
void Comms_SpeakArm(const cavetalk_Mode mode);
void Comms_SpeakAcceleration(const cavetalk_Acceleration *const acceleration);
void Comms_SpeakGyroscope(const cavetalk_Gyroscope *const gyroscope);
void Comms_SpeakEncoders(cavetalk_Encoder *const encoders, const size_t count);


#endif /* COMMS_H */