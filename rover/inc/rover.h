#ifndef ROVER_H
#define ROVER_H

#include <stdbool.h>

#include "bsp.h"

#define ROVER_DEGREES_TO_RADIANS (double)(3.14159265358979323846 / 180.0)

typedef double Rover_Meter_t;
typedef double Rover_MetersPerSecond_t;
typedef double Rover_MetersPerSecondSquared_t;
typedef double Rover_Radian_t;
typedef double Rover_RadiansPerSecond_t;
typedef double RoverImu_AxisCalc_t;

typedef enum
{
    ROVER_ERROR_NONE,
    ROVER_ERROR_NULL,
    ROVER_ERROR_BSP,
    ROVER_ERROR_PERIPHERAL,
    ROVER_ERROR_MODE
} Rover_Error_t;

typedef struct
{
    Rover_MetersPerSecondSquared_t x;
    Rover_MetersPerSecondSquared_t y;
    Rover_MetersPerSecondSquared_t z;
} Rover_AccelerometerReading_t;

typedef struct
{
    Rover_RadiansPerSecond_t x;
    Rover_RadiansPerSecond_t y;
    Rover_RadiansPerSecond_t z;
} Rover_GyroscopeReading_t;

void Rover_Initialize(void);
void Rover_Task(void);
Rover_Error_t Rover_BspToRoverError(const Bsp_Error_t bsp_error);
Rover_Error_t Rover_Arm(void);
Rover_Error_t Rover_Dearm(void);
bool Rover_IsArmed(void);
Rover_Error_t Rover_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate);
Rover_Error_t Rover_ReadAccelerometer(Rover_AccelerometerReading_t *const reading);
Rover_Error_t Rover_ReadGyroscope(Rover_GyroscopeReading_t *const reading);

#endif /* ROVER_H */