#include "gyroscope.h"

#include "bsp.h"

Bsp_Error_t GyroscopeUser_Initialize(void)
{
    return BSP_ERROR_PERIPHERAL;
}

Bsp_Error_t GyroscopeUser_Read(Gyroscope_Reading_t *const reading)
{
    BSP_UNUSED(reading);

    return BSP_ERROR_PERIPHERAL;
}

Bsp_Error_t GyroscopeUser_ReadQuaternion(Gyroscope_Quaternion_t *const quaternion)
{
    BSP_UNUSED(quaternion);

    return BSP_ERROR_PERIPHERAL;
}