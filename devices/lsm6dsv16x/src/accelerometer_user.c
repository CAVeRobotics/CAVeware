#include "bsp.h"

#include "accelerometer.h"

#include "lsm6dsv16x.h"

Bsp_Error_t AccelerometerUser_Initialize(void)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (!Lsm6dsv16x_IsInitialized())
    {
        error = Lsm6dsv16x_Initialize();
    }

    return error;
}

Bsp_Error_t AccelerometerUser_Read(Accelerometer_Reading_t *const reading)
{
    return Lsm6dsv16x_ReadAccelerometer(reading);
}