#include "gyroscope.h"

#include "bsp.h"

Bsp_Error_t Gyroscope_Initialize(Gyroscope_Handle_t *const handle)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = handle->initialize(handle->context);
    }

    return error;
}

Bsp_Error_t Gyroscope_Read(Gyroscope_Handle_t *const handle, Gyroscope_Reading_t *const reading)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = handle->read(handle->context, reading);
    }

    return error;
}

Bsp_Error_t Gyroscope_ReadQuaternion(Gyroscope_Handle_t *const handle, Gyroscope_Quaternion_t *const quaternion)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != handle)
    {
        error = handle->read_quaternion(handle->context, quaternion);
    }

    return error;
}