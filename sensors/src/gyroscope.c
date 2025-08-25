#include "gyroscope.h"

#include "bsp.h"

extern Bsp_Error_t GyroscopeUser_Initialize(void);
extern Bsp_Error_t GyroscopeUser_Read(Gyroscope_Reading_t *const reading);
extern Bsp_Error_t GyroscopeUser_ReadQuaternion(Gyroscope_Quaternion_t *const quaternion);


Bsp_Error_t Gyroscope_Initialize(void)
{
    return GyroscopeUser_Initialize();
}

Bsp_Error_t Gyroscope_Read(Gyroscope_Reading_t *const reading)
{
    return GyroscopeUser_Read(reading);
}

Bsp_Error_t Gyroscope_ReadQuaternion(Gyroscope_Quaternion_t *const quaternion)
{
    return GyroscopeUser_ReadQuaternion(quaternion);
}