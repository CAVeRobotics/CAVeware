#ifndef ROVER_H
#define ROVER_H

#include <stdbool.h>

#include "bsp.h"

#define ROVER_DEGREES_TO_RADIANS (double)(3.14159265358979323846 / 180.0)

typedef double Rover_Meter_t;
typedef double Rover_MetersPerSecond_t;
typedef double Rover_Radian_t;
typedef double Rover_RadiansPerSecond_t;

typedef enum
{
    ROVER_ERROR_NONE,
    ROVER_ERROR_NULL,
    ROVER_ERROR_BSP,
    ROVER_ERROR_PERIPHERAL,
    ROVER_ERROR_MODE
} Rover_Error_t;

void Rover_Initialize(void);
void Rover_Task(void);
Rover_Error_t Rover_BspToRoverError(const Bsp_Error_t bsp_error);
Rover_Error_t Rover_Arm(void);
Rover_Error_t Rover_Dearm(void);
bool Rover_IsArmed(void);
Rover_Error_t Rover_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate);

#endif /* ROVER_H */