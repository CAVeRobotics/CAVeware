#ifndef ROVER_H
#define ROVER_H

#include <stdbool.h>

#include "bsp.h"

#define ROVER_DEGREES_TO_RADIANS (double)(3.14159265358979323846 / 180.0)

typedef uint64_t Rover_Microsecond_t;
typedef double   Rover_Meter_t;
typedef double   Rover_MetersPerSecond_t;
typedef double   Rover_MetersPerSecondSquared_t;
typedef double   Rover_Radian_t;
typedef double   Rover_RadiansPerSecond_t;

typedef enum
{
    ROVER_ERROR_NONE,
    ROVER_ERROR_NULL,
    ROVER_ERROR_BSP,
    ROVER_ERROR_PERIPHERAL,
    ROVER_ERROR_MODE,
    ROVER_ERROR_VALUE
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

typedef struct
{
    double w;
    double x;
    double y;
    double z;
} Rover_Quaternion_t;

void Rover_Initialize(void);
void Rover_Task(void);
Rover_Error_t Rover_BspToRoverError(const Bsp_Error_t bsp_error);
Rover_Error_t Rover_Arm(void);
Rover_Error_t Rover_Dearm(void);
bool Rover_IsArmed(void);
Rover_Error_t Rover_EnableControl(void);
Rover_Error_t Rover_DisableControl(void);
Rover_Error_t Rover_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate);
Rover_Error_t Rover_ReadAccelerometer(Rover_AccelerometerReading_t *const reading);
Rover_Error_t Rover_ReadGyroscope(Rover_GyroscopeReading_t *const reading);
Rover_Error_t Rover_ReadQuaternion(Rover_Quaternion_t *const quaternion);

#endif /* ROVER_H */