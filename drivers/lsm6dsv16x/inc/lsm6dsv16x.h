#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <stdbool.h>
#include <stdint.h>

#include "lsm6dsv16x_reg.h"

#include "bsp.h"

#include "accelerometer.h"
#include "gyroscope.h"

typedef struct
{
    const stmdev_ctx_t interface;
    bool initialized;
} Lsm6dsv16x_Context_t;

extern int32_t Lsm6dsv16x_Write(void *const handle, const uint8_t imu_register, const uint8_t *const data, const uint16_t size);
extern int32_t Lsm6dsv16x_Read(void *const handle, const uint8_t imu_register, uint8_t *const data, const uint16_t size);

#define LSM6DSV16X_INSTANTIATE(spi_handle) \
    {                                      \
        .interface = {                     \
            .write_reg = Lsm6dsv16x_Write, \
            .read_reg = Lsm6dsv16x_Read,   \
            .mdelay = Bsp_Delay,           \
            .handle = spi_handle,          \
        },                                 \
        .initialized = false,              \
    }

Bsp_Error_t Lsm6dsv16x_Initialize(Lsm6dsv16x_Context_t *const context);
bool Lsm6dsv16x_IsInitialized(const Lsm6dsv16x_Context_t *const context);
Bsp_Error_t Lsm6dsv16x_Calibrate(const Lsm6dsv16x_Context_t *const context);
Bsp_Error_t Lsm6dsv16x_ReadAccelerometer(const Lsm6dsv16x_Context_t *const context, Accelerometer_Reading_t *const reading);
Bsp_Error_t Lsm6dsv16x_ReadGyroscope(const Lsm6dsv16x_Context_t *const context, Gyroscope_Reading_t *const reading);
Bsp_Error_t Lsm6dsv16x_ReadQuaterion(const Lsm6dsv16x_Context_t *const context, Gyroscope_Quaternion_t *const quaternion);

#endif /* LSM6DSV16X_H */