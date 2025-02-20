#ifndef ROVER_H
#define ROVER_H

#define ROVER_DEGREES_TO_RADIANS (double)(3.14159265358979323846 / 180.0)

typedef double Rover_Meter_t;
typedef double Rover_MetersPerSecond_t;
typedef double Rover_Radian_t;

typedef enum
{
    ROVER_ERROR_NONE,
    ROVER_ERROR_NULL,
    ROVER_ERROR_BSP
} Rover_Error_t;

#endif /* ROVER_H */