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

typedef enum
{
    ROVER_MODE_STARTUP,
    ROVER_MODE_CONFIGURE,
    ROVER_MODE_RUN,
} Rover_Mode_t;

void Rover_Initialize(void);
void Rover_Task(void);
Rover_Error_t Rover_BspToRoverError(const Bsp_Error_t bsp_error);
Rover_Mode_t Rover_GetMode(void);
Rover_Error_t Rover_SetMode(const Rover_Mode_t mode);

#endif /* ROVER_H */