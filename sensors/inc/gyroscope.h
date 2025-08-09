#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "bsp.h"

typedef struct
{
    Bsp_RadiansPerSecond_t x;
    Bsp_RadiansPerSecond_t y;
    Bsp_RadiansPerSecond_t z;
} Gyroscope_Reading_t;

Bsp_Error_t Gyroscope_Initialize(void);
Bsp_Error_t Gyroscope_Read(Gyroscope_Reading_t *const reading);

#endif /* GYROSCOPE_H */