#include "accelerometer.h"

#include "bsp.h"

extern Bsp_Error_t AccelerometerUser_Initialize(void);
extern Bsp_Error_t AccelerometerUser_Read(Accelerometer_Reading_t *const reading);

Bsp_Error_t Accelerometer_Initialize(void)
{
    return AccelerometerUser_Initialize();
}

Bsp_Error_t Accelerometer_Read(Accelerometer_Reading_t *const reading)
{
    return AccelerometerUser_Read(reading);
}