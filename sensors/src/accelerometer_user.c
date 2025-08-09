#include "accelerometer.h"

#include "bsp.h"

Bsp_Error_t AccelerometerUser_Initialize(void)
{
    return BSP_ERROR_PERIPHERAL;
}

Bsp_Error_t AccelerometerUser_Read(Accelerometer_Reading_t *const reading)
{
    BSP_UNUSED(reading);

    return BSP_ERROR_PERIPHERAL;
}