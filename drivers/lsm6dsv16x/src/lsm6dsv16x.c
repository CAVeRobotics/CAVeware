#include "lsm6dsv16x.h"

#include <stdbool.h>
#include <stdint.h>

#include "lsm6dsv16x_reg.h"
#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"

#include "accelerometer.h"
#include "gyroscope.h"

#define LSM6DSV16X_BOOT_TIME (Bsp_Millisecond_t)10U
#define LSM6DSV16X_TIMEOUT (Bsp_Millisecond_t)10U
#define LSM6DSV16X_CALIBRATION_SETTLE_TIME (Bsp_Millisecond_t)100U
#define LSM6DSV16X_REGISTER_READ 0x80U
#define LSM6DSV16X_FS2_TO_METERS_PER_SECOND_SQUARED (double)5.985e-4
#define LSM6DSV16X_125DPS_TO_RADIANS_PER_SECOND (double)6.658e-5
#define LSM6DSV16X_ERROR_NONE (int32_t)0
#define LSM6DSV16X_MILLIG_TO_G 1e3

typedef int32_t Lsm6dsv16x_Error_t;
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

typedef enum
{
    LSM6DSV16X_FIFO_DATA_X = 0U,
    LSM6DSV16X_FIFO_DATA_Y = 2U,
    LSM6DSV16X_FIFO_DATA_Z = 4U,
    LSM6DSV16X_FIFO_DATA_MAX = 6U
} Lsm6dsv16x_FifoData_t;

static const char *kLsm6dsv16x_LogTag = "LSM6DSV16X";

int32_t Lsm6dsv16x_Write(void *const handle, const uint8_t imu_register, const uint8_t *const data, const uint16_t size);
int32_t Lsm6dsv16x_Read(void *const handle, const uint8_t imu_register, uint8_t *const data, const uint16_t size);
static Bsp_Error_t Lsm6dsv16x_ReadAll(const Lsm6dsv16x_Context_t *const context);
static Bsp_Error_t Lsm6dsv16x_ReadFifo(const Lsm6dsv16x_Context_t *const context);
static inline Bsp_Error_t Lsm6dsv16x_ImuToBspError(const Lsm6dsv16x_Error_t error);
static inline float Lsm6dsv16x_FsToMilliG(const Lsm6dsv16x_Context_t *const context, const Lsm6dsv16x_RawData_t fs);
static inline Bsp_MetersPerSecondSquared_t Lsm6dsv16x_Fs2ToMetersPerSecondSquared(const Lsm6dsv16x_RawData_t fs2);
static inline Bsp_MetersPerSecondSquared_t Lsm6dsv16x_125dpsToRadiansPerSecond(const Lsm6dsv16x_RawData_t dps);

/* Third-party code to handle SFLP quaterion conversion
   Non-compliant, DO NOT MODIFY
   Source: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c
 */
static float_t npy_half_to_float(uint16_t h);
static void sflp2q(float_t quat[4], const uint16_t sflp[3]);

static Lsm6dsv16x_RawData_t Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_MAX] = {
    [LSM6DSV16X_AXIS_X] = 0,
    [LSM6DSV16X_AXIS_Y] = 0,
    [LSM6DSV16X_AXIS_Z] = 0};
static Lsm6dsv16x_RawData_t Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_MAX] = {
    [LSM6DSV16X_AXIS_X] = 0,
    [LSM6DSV16X_AXIS_Y] = 0,
    [LSM6DSV16X_AXIS_Z] = 0};
static double Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_MAX] = {
    [LSM6DSV16X_QUATERION_AXIS_W] = 0.0,
    [LSM6DSV16X_QUATERION_AXIS_X] = 0.0,
    [LSM6DSV16X_QUATERION_AXIS_Y] = 0.0,
    [LSM6DSV16X_QUATERION_AXIS_Z] = 0.0};

/* Onboard registers may not have enough precision to store offset */
static Lsm6dsv16x_RawData_t Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_MAX] = {
    [LSM6DSV16X_AXIS_X] = 0,
    [LSM6DSV16X_AXIS_Y] = 0,
    [LSM6DSV16X_AXIS_Z] = 0};
static Lsm6dsv16x_RawData_t Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_MAX] = {
    [LSM6DSV16X_AXIS_X] = 0,
    [LSM6DSV16X_AXIS_Y] = 0,
    [LSM6DSV16X_AXIS_Z] = 0};

Bsp_Error_t Lsm6dsv16x_Initialize(Lsm6dsv16x_Context_t *const context)
{
    Lsm6dsv16x_Error_t error = LSM6DSV16X_ERROR_NONE;
    Lsm6dsv16x_Error_t unused_var;

    /* Wait sensor boot time */
    Bsp_Delay(LSM6DSV16X_BOOT_TIME);

    /* Check device ID */
    uint8_t whoami = 0U;
    if (NULL == context)
    {
        error = 1;

        BSP_LOGGER_LOG_ERROR(kLsm6dsv16x_LogTag, "Context is NULL");
    }
    else if (context->initialized)
    {
        /* Do nothing */
    }
    else if ((0 != lsm6dsv16x_device_id_get(&context->interface, &whoami)) || (LSM6DSV16X_ID != whoami))
    {
        error = 1;

        BSP_LOGGER_LOG_ERROR(kLsm6dsv16x_LogTag, "Failed to detect IMU");
    }
    else
    {
        BSP_LOGGER_LOG_DEBUG(kLsm6dsv16x_LogTag, "IMU detected");

        /* Restore default configuration */
        BSP_LOGGER_LOG_DEBUG(kLsm6dsv16x_LogTag, "Resetting IMU");
        lsm6dsv16x_reset_t imu_reset;
        error |= lsm6dsv16x_reset_set(&context->interface, LSM6DSV16X_RESTORE_CTRL_REGS);
        error |= lsm6dsv16x_reset_get(&context->interface, &imu_reset);
        while (LSM6DSV16X_READY != imu_reset)
        {
            error |= lsm6dsv16x_reset_get(&context->interface, &imu_reset);
        }
        BSP_LOGGER_LOG_DEBUG(kLsm6dsv16x_LogTag, "IMU reset");

        /* Enable Block Data Update */
        error |= lsm6dsv16x_block_data_update_set(&context->interface, PROPERTY_ENABLE);

        /* Set Output Data Rate.
         * Selected data rate have to be equal or greater with respect
         * with MLC data rate.
         */
        error |= lsm6dsv16x_xl_data_rate_set(&context->interface, LSM6DSV16X_ODR_AT_7680Hz);
        error |= lsm6dsv16x_gy_data_rate_set(&context->interface, LSM6DSV16X_ODR_AT_7680Hz);

        /* Set full scale */
        error |= lsm6dsv16x_xl_full_scale_set(&context->interface, LSM6DSV16X_2g);
        error |= lsm6dsv16x_gy_full_scale_set(&context->interface, LSM6DSV16X_125dps);

        /* Configure filtering chain */
        lsm6dsv16x_filt_settling_mask_t filter_settling_mask;
        filter_settling_mask.drdy = PROPERTY_ENABLE;
        filter_settling_mask.irq_xl = PROPERTY_ENABLE;
        filter_settling_mask.irq_g = PROPERTY_ENABLE;
        error |= lsm6dsv16x_filt_settling_mask_set(&context->interface, filter_settling_mask);
        error |= lsm6dsv16x_filt_gy_lp1_set(&context->interface, PROPERTY_ENABLE);
        error |= lsm6dsv16x_filt_gy_lp1_bandwidth_set(&context->interface, LSM6DSV16X_GY_ULTRA_LIGHT);
        error |= lsm6dsv16x_filt_xl_lp2_set(&context->interface, PROPERTY_ENABLE);
        error |= lsm6dsv16x_filt_xl_lp2_bandwidth_set(&context->interface, LSM6DSV16X_XL_STRONG);

        /* Enable game rotation vector */
        lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
        fifo_sflp.game_rotation = 1;
        fifo_sflp.gravity = 0;
        fifo_sflp.gbias = 0;
        error |= lsm6dsv16x_fifo_sflp_batch_set(&context->interface, fifo_sflp);

        /* Set FIFO mode to Continuous mode */
        error |= lsm6dsv16x_fifo_mode_set(&context->interface, LSM6DSV16X_STREAM_MODE);

        /* Set Game Vector Output Data Rate */
        error |= lsm6dsv16x_sflp_data_rate_set(&context->interface, LSM6DSV16X_SFLP_480Hz);
        error |= lsm6dsv16x_sflp_game_rotation_set(&context->interface, PROPERTY_ENABLE);

        if (BSP_ERROR_NONE != Lsm6dsv16x_Calibrate(context))
        {
            error = 1;
        }
        else
        {
            context->initialized = true;

            BSP_LOGGER_LOG_DEBUG(kLsm6dsv16x_LogTag, "Initialized");
        }
    }

    if (LSM6DSV16X_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kLsm6dsv16x_LogTag, "Failed to initialize");
    }

    return Lsm6dsv16x_ImuToBspError(error);
}

bool Lsm6dsv16x_IsInitialized(const Lsm6dsv16x_Context_t *const context)
{
    bool initialized = false;

    if (NULL != context)
    {
        initialized = context->initialized;
    }

    return initialized;
}

Bsp_Error_t Lsm6dsv16x_Calibrate(const Lsm6dsv16x_Context_t *const context)
{
    Lsm6dsv16x_Error_t error = LSM6DSV16X_ERROR_NONE;
    lsm6dsv16x_fifo_status_t fifo_status = {0U};
    uint16_t xl_samples = 0U;
    uint16_t gy_samples = 0U;
    int32_t xl_data[LSM6DSV16X_AXIS_MAX] = {0U};
    int32_t gy_data[LSM6DSV16X_AXIS_MAX] = {0U};
    int32_t gravity_offset = (int32_t)(LSM6DSV16X_MILLIG_TO_G / Lsm6dsv16x_FsToMilliG(context, 1));
    lsm6dsv16x_data_rate_t xl_data_rate;
    lsm6dsv16x_data_rate_t gy_data_rate;
    lsm6dsv16x_fifo_xl_batch_t xl_fifo_batch;
    lsm6dsv16x_fifo_gy_batch_t gy_fifo_batch;
    lsm6dsv16x_fifo_mode_t fifo_mode;
    lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
    lsm6dsv16x_sflp_data_rate_t sflp_data_rate;
    uint8_t game_rotation_enabled;

    BSP_LOGGER_LOG_DEBUG(kLsm6dsv16x_LogTag, "Calibrating");

    if (NULL == context)
    {
        error = 1;

        BSP_LOGGER_LOG_ERROR(kLsm6dsv16x_LogTag, "Context is NULL");
    }
    else
    {
        /* Save existing data rate */
        error |= lsm6dsv16x_xl_data_rate_get(&context->interface, &xl_data_rate);
        error |= lsm6dsv16x_gy_data_rate_get(&context->interface, &gy_data_rate);
        error |= lsm6dsv16x_fifo_xl_batch_get(&context->interface, &xl_fifo_batch);
        error |= lsm6dsv16x_fifo_gy_batch_get(&context->interface, &gy_fifo_batch);

        /* Save exisitng FIFO mode */
        error |= lsm6dsv16x_fifo_mode_get(&context->interface, &fifo_mode);

        /* Save existing SFLP settings */
        error |= lsm6dsv16x_fifo_sflp_batch_get(&context->interface, &fifo_sflp);
        error |= lsm6dsv16x_sflp_data_rate_get(&context->interface, &sflp_data_rate);
        error |= lsm6dsv16x_sflp_game_rotation_get(&context->interface, &game_rotation_enabled);

        /* Set new data rate for calibration period (1s~2s) to fill FIFO */
        error |= lsm6dsv16x_xl_data_rate_set(&context->interface, LSM6DSV16X_ODR_AT_240Hz);
        error |= lsm6dsv16x_gy_data_rate_set(&context->interface, LSM6DSV16X_ODR_AT_240Hz);
        error |= lsm6dsv16x_fifo_xl_batch_set(&context->interface, LSM6DSV16X_XL_BATCHED_AT_240Hz);
        error |= lsm6dsv16x_fifo_gy_batch_set(&context->interface, LSM6DSV16X_GY_BATCHED_AT_240Hz);

        /* Disable SFLP */
        lsm6dsv16x_fifo_sflp_raw_t fifo_sflp_disabled;
        fifo_sflp_disabled.game_rotation = 0;
        fifo_sflp_disabled.gravity = 0;
        fifo_sflp_disabled.gbias = 0;
        error |= lsm6dsv16x_fifo_sflp_batch_set(&context->interface, fifo_sflp_disabled);
        error |= lsm6dsv16x_sflp_game_rotation_set(&context->interface, PROPERTY_DISABLE);

        Bsp_Delay(LSM6DSV16X_CALIBRATION_SETTLE_TIME);

        /* Flush FIFO */
        error |= lsm6dsv16x_fifo_status_get(&context->interface, &fifo_status);
        for (uint16_t i = 0U; i < fifo_status.fifo_level; i++)
        {
            lsm6dsv16x_fifo_out_raw_t fifo_data;
            error |= lsm6dsv16x_fifo_out_raw_get(&context->interface, &fifo_data);
        }

        /* Set FIFO to FIFO mode */
        error |= lsm6dsv16x_fifo_mode_set(&context->interface, LSM6DSV16X_FIFO_MODE);

        /* Wait until FIFO is full */
        while (0U == fifo_status.fifo_full)
        {
            error |= lsm6dsv16x_fifo_status_get(&context->interface, &fifo_status);
        }

        /* Read and average FIFO samples */
        for (uint16_t i = 0U; i < fifo_status.fifo_level; i++)
        {
            lsm6dsv16x_fifo_out_raw_t fifo_data;
            error |= lsm6dsv16x_fifo_out_raw_get(&context->interface, &fifo_data);

            Lsm6dsv16x_RawData_t x = *(Lsm6dsv16x_RawData_t *)&fifo_data.data[LSM6DSV16X_FIFO_DATA_X];
            Lsm6dsv16x_RawData_t y = *(Lsm6dsv16x_RawData_t *)&fifo_data.data[LSM6DSV16X_FIFO_DATA_Y];
            Lsm6dsv16x_RawData_t z = *(Lsm6dsv16x_RawData_t *)&fifo_data.data[LSM6DSV16X_FIFO_DATA_Z];

            switch (fifo_data.tag)
            {
            case LSM6DSV16X_XL_NC_TAG:
                xl_data[LSM6DSV16X_AXIS_X] += (int32_t)x;
                xl_data[LSM6DSV16X_AXIS_Y] += (int32_t)y;
                xl_data[LSM6DSV16X_AXIS_Z] += (int32_t)z;
                xl_samples++;
                break;
            case LSM6DSV16X_GY_NC_TAG:
                gy_data[LSM6DSV16X_AXIS_X] += (int32_t)x;
                gy_data[LSM6DSV16X_AXIS_Y] += (int32_t)y;
                gy_data[LSM6DSV16X_AXIS_Z] += (int32_t)z;
                gy_samples++;
                break;
            default:
                break;
            }
        }

        /* Calculate accelerometer offset */
        Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_X] = (int16_t)(xl_data[LSM6DSV16X_AXIS_X] / (int32_t)xl_samples);
        Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_Y] = (int16_t)(xl_data[LSM6DSV16X_AXIS_Y] / (int32_t)xl_samples);
        Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_Z] = (int16_t)((xl_data[LSM6DSV16X_AXIS_Z] / (int32_t)xl_samples) - gravity_offset);

        /* Calculate gyroscope offset */
        Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_X] = (int16_t)(gy_data[LSM6DSV16X_AXIS_X] / (int32_t)gy_samples);
        Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_Y] = (int16_t)(gy_data[LSM6DSV16X_AXIS_Y] / (int32_t)gy_samples);
        Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_Z] = (int16_t)(gy_data[LSM6DSV16X_AXIS_Z] / (int32_t)gy_samples);

        /* Restore data rate */
        error |= lsm6dsv16x_xl_data_rate_set(&context->interface, xl_data_rate);
        error |= lsm6dsv16x_gy_data_rate_set(&context->interface, gy_data_rate);
        error |= lsm6dsv16x_fifo_xl_batch_set(&context->interface, xl_fifo_batch);
        error |= lsm6dsv16x_fifo_gy_batch_set(&context->interface, gy_fifo_batch);

        /* Restore FIFO mode */
        error |= lsm6dsv16x_fifo_mode_set(&context->interface, fifo_mode);

        /* Restore SFLP settings */
        error |= lsm6dsv16x_fifo_sflp_batch_set(&context->interface, fifo_sflp);
        error |= lsm6dsv16x_sflp_data_rate_set(&context->interface, sflp_data_rate);
        error |= lsm6dsv16x_sflp_game_rotation_set(&context->interface, game_rotation_enabled);
    }

    if (LSM6DSV16X_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kLsm6dsv16x_LogTag, "Calibration failed");
    }
    else
    {
        BSP_LOGGER_LOG_DEBUG(kLsm6dsv16x_LogTag, "Calibrated");
    }

    return Lsm6dsv16x_ImuToBspError(error);
}

Bsp_Error_t Lsm6dsv16x_ReadAccelerometer(const Lsm6dsv16x_Context_t *const context, Accelerometer_Reading_t *const reading)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if ((NULL != context) && (NULL != reading))
    {
        error = Lsm6dsv16x_ReadAll(context);

        reading->x = Lsm6dsv16x_Fs2ToMetersPerSecondSquared(Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_X]);
        reading->y = Lsm6dsv16x_Fs2ToMetersPerSecondSquared(Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_Y]);
        reading->z = Lsm6dsv16x_Fs2ToMetersPerSecondSquared(Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_Z]);
    }

    return error;
}

Bsp_Error_t Lsm6dsv16x_ReadGyroscope(const Lsm6dsv16x_Context_t *const context, Gyroscope_Reading_t *const reading)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if ((NULL != context) && (NULL != reading))
    {
        error = Lsm6dsv16x_ReadAll(context);

        reading->x = Lsm6dsv16x_125dpsToRadiansPerSecond(Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_X]);
        reading->y = Lsm6dsv16x_125dpsToRadiansPerSecond(Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_Y]);
        reading->z = Lsm6dsv16x_125dpsToRadiansPerSecond(Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_Z]);
    }

    return error;
}

Bsp_Error_t Lsm6dsv16x_ReadQuaterion(const Lsm6dsv16x_Context_t *const context, Gyroscope_Quaternion_t *const quaternion)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if ((context != NULL) && (NULL != quaternion))
    {
        error = Lsm6dsv16x_ReadFifo(context);

        quaternion->w = Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_W];
        quaternion->x = Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_X];
        quaternion->y = Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_Y];
        quaternion->z = Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_Z];
    }

    return error;
}

Lsm6dsv16x_Error_t Lsm6dsv16x_Write(void *const handle, const uint8_t imu_register, const uint8_t *const data, const uint16_t size)
{
    Lsm6dsv16x_Error_t error = LSM6DSV16X_ERROR_NONE;

    if ((BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_RESET)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Transmit(handle, &imu_register, 1U, LSM6DSV16X_TIMEOUT)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Transmit(handle, data, size, LSM6DSV16X_TIMEOUT)) ||
        (BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_SET)))
    {
        error = 1;
    }

    return error;
}

Lsm6dsv16x_Error_t Lsm6dsv16x_Read(void *const handle, const uint8_t imu_register, uint8_t *const data, const uint16_t size)
{
    Lsm6dsv16x_Error_t error = LSM6DSV16X_ERROR_NONE;
    uint8_t register_read = imu_register | LSM6DSV16X_REGISTER_READ;

    if ((BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_RESET)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Transmit(handle, &register_read, 1U, LSM6DSV16X_TIMEOUT)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Receive(handle, data, size, LSM6DSV16X_TIMEOUT)) ||
        (BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_SET)))
    {
        error = 1;
    }

    return error;
}

static Bsp_Error_t Lsm6dsv16x_ReadAll(const Lsm6dsv16x_Context_t *const context)
{
    Lsm6dsv16x_Error_t error = LSM6DSV16X_ERROR_NONE;
    lsm6dsv16x_data_ready_t data_ready;

    error |= lsm6dsv16x_flag_data_ready_get(&context->interface, &data_ready);

    if (data_ready.drdy_xl)
    {
        error |= lsm6dsv16x_acceleration_raw_get(&context->interface, Lsm6dsv16x_RawAccelerometer);

        Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_X] -= Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_X];
        Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_Y] -= Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_Y];
        Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_Z] -= Lsm6dsv16x_AccelerometerOffset[LSM6DSV16X_AXIS_Z];

        /* Correct for IMU orientation in bot */
        Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_X] *= -1;
        Lsm6dsv16x_RawAccelerometer[LSM6DSV16X_AXIS_Y] *= -1;
    }

    if (data_ready.drdy_gy)
    {
        error |= lsm6dsv16x_angular_rate_raw_get(&context->interface, Lsm6dsv16x_RawGyroscope);

        Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_X] -= Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_X];
        Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_Y] -= Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_Y];
        Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_Z] -= Lsm6dsv16x_GyroscopeOffset[LSM6DSV16X_AXIS_Z];

        /* Correct for IMU orientation in bot */
        Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_X] *= -1;
        Lsm6dsv16x_RawGyroscope[LSM6DSV16X_AXIS_Y] *= -1;
    }

    return Lsm6dsv16x_ImuToBspError(error);
}

static Bsp_Error_t Lsm6dsv16x_ReadFifo(const Lsm6dsv16x_Context_t *const context)
{
    lsm6dsv16x_fifo_status_t fifo_status;
    float_t quaternion[LSM6DSV16X_QUATERION_AXIS_MAX] = {
        0U};
    float_t quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_MAX] = {
        0U};
    uint16_t game_rotation_vector_samples = 0U;
    Lsm6dsv16x_Error_t error = lsm6dsv16x_fifo_status_get(&context->interface, &fifo_status);

    for (uint16_t i = 0U; i < fifo_status.fifo_level; i++)
    {
        lsm6dsv16x_fifo_out_raw_t fifo_data;
        error |= lsm6dsv16x_fifo_out_raw_get(&context->interface, &fifo_data);

        switch (fifo_data.tag)
        {
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
            sflp2q(quaternion, (uint16_t *)&fifo_data.data[0]);
            quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_W] += quaternion[LSM6DSV16X_QUATERION_AXIS_W];
            quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_X] += quaternion[LSM6DSV16X_QUATERION_AXIS_X];
            quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_Y] += quaternion[LSM6DSV16X_QUATERION_AXIS_Y];
            quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_Z] += quaternion[LSM6DSV16X_QUATERION_AXIS_Z];
            game_rotation_vector_samples++;
        default:
            break;
        }
    }

    Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_W] = (double)quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_W] / game_rotation_vector_samples;
    Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_X] = (double)quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_X] / game_rotation_vector_samples;
    Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_Y] = (double)quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_Y] / game_rotation_vector_samples;
    Lsm6dsv16x_Quaternion[LSM6DSV16X_QUATERION_AXIS_Z] = (double)quaternion_averaged[LSM6DSV16X_QUATERION_AXIS_Z] / game_rotation_vector_samples;

    return Lsm6dsv16x_ImuToBspError(error);
}

static inline Bsp_Error_t Lsm6dsv16x_ImuToBspError(const Lsm6dsv16x_Error_t error)
{
    Bsp_Error_t bsp_error = BSP_ERROR_NONE;

    if (LSM6DSV16X_ERROR_NONE != error)
    {
        bsp_error = BSP_ERROR_PERIPHERAL;
    }

    return bsp_error;
}

static inline float Lsm6dsv16x_FsToMilliG(const Lsm6dsv16x_Context_t *const context, const Lsm6dsv16x_RawData_t fs)
{
    float mg = 0.0f;
    lsm6dsv16x_xl_full_scale_t xl_full_scale;

    (void)lsm6dsv16x_xl_full_scale_get(&context->interface, &xl_full_scale);

    switch (xl_full_scale)
    {
    case LSM6DSV16X_2g:
        mg = lsm6dsv16x_from_fs2_to_mg(fs);
        break;
    case LSM6DSV16X_4g:
        mg = lsm6dsv16x_from_fs4_to_mg(fs);
        break;
    case LSM6DSV16X_8g:
        mg = lsm6dsv16x_from_fs8_to_mg(fs);
        break;
    case LSM6DSV16X_16g:
        mg = lsm6dsv16x_from_fs16_to_mg(fs);
        break;
    default:
        break;
    }

    return mg;
}

static inline Bsp_MetersPerSecondSquared_t Lsm6dsv16x_Fs2ToMetersPerSecondSquared(const Lsm6dsv16x_RawData_t fs2)
{
    return fs2 * LSM6DSV16X_FS2_TO_METERS_PER_SECOND_SQUARED;
}

static inline Bsp_MetersPerSecondSquared_t Lsm6dsv16x_125dpsToRadiansPerSecond(const Lsm6dsv16x_RawData_t dps)
{
    return dps * LSM6DSV16X_125DPS_TO_RADIANS_PER_SECOND;
}

/* Third-party code to handle SFLP quaterion conversion
   Non-compliant, DO NOT MODIFY
   Source: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c
 */
static float_t npy_half_to_float(uint16_t h)
{
    union
    {
        float_t ret;
        uint32_t retbits;
    } conv;
    conv.retbits = lsm6dsv16x_from_f16_to_f32(h);
    return conv.ret;
}

/* Third-party code to handle SFLP quaterion conversion
   Non-compliant, DO NOT MODIFY
   Source: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c
 */
static void sflp2q(float_t quat[4], const uint16_t sflp[3])
{
    float_t sumsq = 0;

    quat[0] = npy_half_to_float(sflp[0]);
    quat[1] = npy_half_to_float(sflp[1]);
    quat[2] = npy_half_to_float(sflp[2]);

    for (uint8_t i = 0; i < 3; i++)
        sumsq += quat[i] * quat[i];

    if (sumsq > 1.0f)
    {
        float_t n = sqrtf(sumsq);
        quat[0] /= n;
        quat[1] /= n;
        quat[2] /= n;
        sumsq = 1.0f;
    }

    quat[3] = sqrtf(1.0f - sumsq);
}