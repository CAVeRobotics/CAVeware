#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "bsp.h"

typedef struct
{
    Bsp_MetersPerSecondSquared_t x;
    Bsp_MetersPerSecondSquared_t y;
    Bsp_MetersPerSecondSquared_t z;
} Accelerometer_Reading_t;

Bsp_Error_t Accelerometer_Initialize(void);
Bsp_Error_t Accelerometer_Read(Accelerometer_Reading_t *const reading);

#endif /* ACCELEROMETER_H */