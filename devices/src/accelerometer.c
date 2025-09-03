#include "accelerometer.h"

#include "bsp.h"

Bsp_Error_t Accelerometer_Initialize(Accelerometer_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = handle->initialize(handle->context);
    }

    return error;
}

Bsp_Error_t Accelerometer_Read(Accelerometer_Handle_t *const handle, Accelerometer_Reading_t *const reading)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = handle->read(handle->context, reading);
    }

    return error;
}