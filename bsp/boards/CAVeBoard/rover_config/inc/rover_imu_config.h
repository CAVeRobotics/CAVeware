#ifndef ROVER_IMU_CONFIG_H
#define ROVER_IMU_CONFIG_H

#include <stdbool.h>

#include "rover.h"

Rover_Error_t RoverImuConfig_Initialize(void);
Rover_Error_t RoverImuConfig_ReadAccelerometer(Rover_AccelerometerReading_t *const reading);
Rover_Error_t RoverImuConfig_ReadGyroscope(Rover_GyroscopeReading_t *const reading);

#endif /* ROVER_IMU_CONFIG_H */