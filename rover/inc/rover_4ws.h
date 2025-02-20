#ifndef ROVER_4WS_H
#define ROVER_4WS_H

#include "rover.h"

Rover_Error_t Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle);
Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle);

#endif /* ROVER_4WS_H */