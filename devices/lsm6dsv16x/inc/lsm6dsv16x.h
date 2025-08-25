#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <stdbool.h>

#include "bsp.h"

#include "accelerometer.h"
#include "gyroscope.h"

Bsp_Error_t Lsm6dsv16x_Initialize(void);
bool Lsm6dsv16x_IsInitialized(void);
Bsp_Error_t Lsm6dsv16x_Calibrate(void);
Bsp_Error_t Lsm6dsv16x_ReadAccelerometer(Accelerometer_Reading_t *const reading);
Bsp_Error_t Lsm6dsv16x_ReadGyroscope(Gyroscope_Reading_t *const reading);
Bsp_Error_t Lsm6dsv16x_ReadQuaterion(Gyroscope_Quaternion_t *const quaternion);

#endif /* LSM6DSV16X_H */