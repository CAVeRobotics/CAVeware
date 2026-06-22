#ifndef ROVER_4WS_H
#define ROVER_4WS_H

#include <stdbool.h>

#include "bsp.h"

bool Rover4ws_Arm(void);
bool Rover4ws_Disarm(void);
void Rover4ws_EnableSpeedControl(void);
void Rover4ws_DisableSpeedControl(void);
void Rover4ws_EnableSteeringControl(void);
void Rover4ws_DisableSteeringControl(void);
void Rover4ws_Run(void);
void Rover4ws_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);

#endif /* ROVER_4WS_H */