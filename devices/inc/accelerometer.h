#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "bsp.h"

typedef struct
{
    Bsp_MetersPerSecondSquared_t x;
    Bsp_MetersPerSecondSquared_t y;
    Bsp_MetersPerSecondSquared_t z;
} Accelerometer_Reading_t;

typedef struct
{
    void *context;
    Bsp_Error_t (*initialize)(void *const context);
    Bsp_Error_t (*read)(void *const context, Accelerometer_Reading_t *const reading);
} Accelerometer_Handle_t;

Bsp_Error_t Accelerometer_Initialize(Accelerometer_Handle_t *const handle);
Bsp_Error_t Accelerometer_Read(Accelerometer_Handle_t *const handle, Accelerometer_Reading_t *const reading);

#endif /* ACCELEROMETER_H */