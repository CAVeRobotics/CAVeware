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

typedef Bsp_Error_t (*Gyroscope_Initialize_t)(void *const context);
typedef Bsp_Error_t (*Gyroscope_Read_t)(void *const context, Gyroscope_Reading_t *const reading);
typedef Bsp_Error_t (*Gyroscope_ReadQuaternion_t)(void *const context, Gyroscope_Quaternion_t *const quaternion);

typedef struct
{
    void *context;
    Gyroscope_Initialize_t initialize;
    Gyroscope_Read_t read;
    Gyroscope_ReadQuaternion_t read_quaternion;
} Gyroscope_Handle_t;

Bsp_Error_t Gyroscope_Initialize(Gyroscope_Handle_t *const handle);
Bsp_Error_t Gyroscope_Read(Gyroscope_Handle_t *const handle, Gyroscope_Reading_t *const reading);
Bsp_Error_t Gyroscope_ReadQuaternion(Gyroscope_Handle_t *const handle, Gyroscope_Quaternion_t *const quaternion);

#endif /* GYROSCOPE_H */