#include "gyroscope.h"

#include "bsp.h"

extern Bsp_Error_t GyroscopeUser_Initialize(void);
extern Bsp_Error_t GyroscopeUser_Read(Gyroscope_Reading_t *const reading);

Bsp_Error_t Gyroscope_Initialize(void)
{
    return GyroscopeUser_Initialize();
}

Bsp_Error_t Gyroscope_Read(Gyroscope_Reading_t *const reading)
{
    return GyroscopeUser_Read(reading);
}