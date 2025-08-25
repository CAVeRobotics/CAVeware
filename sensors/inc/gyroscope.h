#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "bsp.h"

typedef struct
{
    Bsp_RadiansPerSecond_t x;
    Bsp_RadiansPerSecond_t y;
    Bsp_RadiansPerSecond_t z;
} Gyroscope_Reading_t;

typedef struct
{
    double w;
    double x;
    double y;
    double z;
} Gyroscope_Quaternion_t;

Bsp_Error_t Gyroscope_Initialize(void);
Bsp_Error_t Gyroscope_Read(Gyroscope_Reading_t *const reading);
Bsp_Error_t Gyroscope_ReadQuaternion(Gyroscope_Quaternion_t *const quaternion);

#endif /* GYROSCOPE_H */