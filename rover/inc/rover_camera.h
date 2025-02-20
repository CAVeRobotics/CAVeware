#ifndef ROVER_CAMERA_H
#define ROVER_CAMERA_H

#include "rover.h"

Rover_Error_t RoverCamera_Pan(const Rover_Radian_t pan_angle);
Rover_Error_t RoverCamera_Tilt(const Rover_Radian_t tilt_angle);

#endif /* ROVER_CAMERA_H */