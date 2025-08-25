#include "bsp.h"

#include "gyroscope.h"

#include "lsm6dsv16x.h"

Bsp_Error_t GyroscopeUser_Initialize(void)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (!Lsm6dsv16x_IsInitialized())
    {
        error = Lsm6dsv16x_Initialize();
    }

    return error;
}

Bsp_Error_t GyroscopeUser_Read(Gyroscope_Reading_t *const reading)
{
    return Lsm6dsv16x_ReadGyroscope(reading);
}

Bsp_Error_t GyroscopeUser_ReadQuaternion(Gyroscope_Quaternion_t *const quaternion)
{
    return Lsm6dsv16x_ReadQuaterion(quaternion);
}