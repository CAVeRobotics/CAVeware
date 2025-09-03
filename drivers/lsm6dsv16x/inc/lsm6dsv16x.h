#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <stdbool.h>
#include <stdint.h>

#include "lsm6dsv16x_reg.h"

#include "bsp.h"

#include "accelerometer.h"
#include "gyroscope.h"

typedef int16_t Lsm6dsv16x_RawData_t;

typedef enum
{
    LSM6DSV16X_AXIS_X,
    LSM6DSV16X_AXIS_Y,
    LSM6DSV16X_AXIS_Z,
    LSM6DSV16X_AXIS_MAX
} Lsm6dsv16x_Axis_t;

typedef enum
{
    LSM6DSV16X_QUATERION_AXIS_W,
    LSM6DSV16X_QUATERION_AXIS_X,
    LSM6DSV16X_QUATERION_AXIS_Y,
    LSM6DSV16X_QUATERION_AXIS_Z,
    LSM6DSV16X_QUATERION_AXIS_MAX
} Lsm6dsv16x_QuaterionAxis_t;

typedef struct
{
    const stmdev_ctx_t interface;
    bool initialized;
    Lsm6dsv16x_RawData_t raw_accelerometer[LSM6DSV16X_AXIS_MAX];
    Lsm6dsv16x_RawData_t raw_gyroscope[LSM6DSV16X_AXIS_MAX];
    double quaternion[LSM6DSV16X_QUATERION_AXIS_MAX];
    Lsm6dsv16x_RawData_t accelerometer_offset[LSM6DSV16X_AXIS_MAX]; /* Onboard registers may not have enough precision to store offset */
    Lsm6dsv16x_RawData_t gyroscope_offset[LSM6DSV16X_AXIS_MAX];     /* Onboard registers may not have enough precision to store offset */
} Lsm6dsv16x_Context_t;

extern int32_t Lsm6dsv16x_Write(void *const handle, const uint8_t imu_register, const uint8_t *const data, const uint16_t size);
extern int32_t Lsm6dsv16x_Read(void *const handle, const uint8_t imu_register, uint8_t *const data, const uint16_t size);

Bsp_Error_t Lsm6dsv16x_Initialize(Lsm6dsv16x_Context_t *const context);
bool Lsm6dsv16x_IsInitialized(const Lsm6dsv16x_Context_t *const context);
Bsp_Error_t Lsm6dsv16x_Calibrate(Lsm6dsv16x_Context_t *const context);
Bsp_Error_t Lsm6dsv16x_ReadAccelerometer(Lsm6dsv16x_Context_t *const context, Accelerometer_Reading_t *const reading);
Bsp_Error_t Lsm6dsv16x_ReadGyroscope(Lsm6dsv16x_Context_t *const context, Gyroscope_Reading_t *const reading);
Bsp_Error_t Lsm6dsv16x_ReadQuaterion(Lsm6dsv16x_Context_t *const context, Gyroscope_Quaternion_t *const quaternion);

#define LSM6DSV16X_CONTEXT(spi_handle)               \
        {                                            \
            .interface     = {                       \
                .write_reg = Lsm6dsv16x_Write,       \
                .read_reg  = Lsm6dsv16x_Read,        \
                .mdelay    = Bsp_Delay,              \
                .handle    = spi_handle,             \
            },                                       \
            .initialized            = false,         \
            .accelerometer_offset   = {              \
                [LSM6DSV16X_AXIS_X] = 0,             \
                [LSM6DSV16X_AXIS_Y] = 0,             \
                [LSM6DSV16X_AXIS_Z] = 0,             \
            },                                       \
            .gyroscope_offset       = {              \
                [LSM6DSV16X_AXIS_X] = 0,             \
                [LSM6DSV16X_AXIS_Y] = 0,             \
                [LSM6DSV16X_AXIS_Z] = 0,             \
            },                                       \
            .raw_accelerometer      = {              \
                [LSM6DSV16X_AXIS_X] = 0,             \
                [LSM6DSV16X_AXIS_Y] = 0,             \
                [LSM6DSV16X_AXIS_Z] = 0,             \
            },                                       \
            .raw_gyroscope          = {              \
                [LSM6DSV16X_AXIS_X] = 0,             \
                [LSM6DSV16X_AXIS_Y] = 0,             \
                [LSM6DSV16X_AXIS_Z] = 0,             \
            },                                       \
            .quaternion                       = {    \
                [LSM6DSV16X_QUATERION_AXIS_W] = 0.0, \
                [LSM6DSV16X_QUATERION_AXIS_X] = 0.0, \
                [LSM6DSV16X_QUATERION_AXIS_Y] = 0.0, \
                [LSM6DSV16X_QUATERION_AXIS_Z] = 0.0, \
            },                                       \
        }

#define LSM6DSV16X_ACCELEROMETER_HANDLE(lsm6dsv16x_context)                   \
        {                                                                     \
            .context    = (void *)&lsm6dsv16x_context,                        \
            .initialize = (Accelerometer_Initialize_t)Lsm6dsv16x_Initialize,  \
            .read       = (Accelerometer_Read_t)Lsm6dsv16x_ReadAccelerometer, \
        }

#define LSM6DSV16X_GYROSCOPE_HANDLE(lsm6dsv16x_context)                             \
        {                                                                           \
            .context         = (void *)&lsm6dsv16x_context,                         \
            .initialize      = (Gyroscope_Initialize_t)Lsm6dsv16x_Initialize,       \
            .read            = (Gyroscope_Read_t)Lsm6dsv16x_ReadGyroscope,          \
            .read_quaternion = (Gyroscope_ReadQuaternion_t)Lsm6dsv16x_ReadQuaterion \
        }

#endif /* LSM6DSV16X_H */