#include "rover_imu_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "lsm6dsv16x_reg.h"
#include "spi.h"
#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"

#include "rover.h"

#define ROVER_IMU_CONFIG_BOOT_TIME                        (Bsp_Millisecond_t)10U
#define ROVER_IMU_CONFIG_TIMEOUT                          (Bsp_Millisecond_t)10U
#define ROVER_IMU_CONFIG_CALIBRATION_SETTLE_TIME          (Bsp_Millisecond_t)100U
#define ROVER_IMU_CONFIG_REGISTER_READ                    0x80U
#define ROVER_IMU_CONFIG_FS2_TO_METERS_PER_SECOND_SQUARED (double)5.985e-4
#define ROVER_IMU_CONFIG_125DPS_TO_RADIANS_PER_SECOND     (double)6.658e-5
#define ROVER_IMU_CONFIG_ERROR_NONE                       (int32_t)0
#define ROVER_IMU_MILLIG_TO_G                             1e3

typedef int32_t RoverImuConfig_Error_t;
typedef int16_t RoverImuConfig_RawData_t;

typedef enum
{
    ROVER_IMU_CONFIG_AXIS_X,
    ROVER_IMU_CONFIG_AXIS_Y,
    ROVER_IMU_CONFIG_AXIS_Z,
    ROVER_IMU_CONFIG_AXIS_MAX
} RoverImuConfig_Axis_t;

typedef enum
{
    ROVER_IMU_CONFIG_FIFO_DATA_X = 0U,
    ROVER_IMU_CONFIG_FIFO_DATA_Y = 2U,
    ROVER_IMU_CONFIG_FIFO_DATA_Z = 4U,
    ROVER_IMU_CONFIG_FIFO_DATA_MAX = 6U
} RoverImuConfig_FifoData_t;

static const char *kRoverImuConfig_LogTag = "ROVER IMU CONFIG";

static int32_t RoverImuConfig_Write(void *const handle, const uint8_t imu_register, const uint8_t *const data, const uint16_t size);
static int32_t RoverImuConfig_Read(void *const handle, const uint8_t imu_register, uint8_t *const data, const uint16_t size);
static Rover_Error_t RoverImuConfig_ReadAll(void);
static inline Rover_Error_t RoverImuConfig_ImuToRoverError(const RoverImuConfig_Error_t error);
static inline float RoverImuConfig_FsToMilliG(const RoverImuConfig_RawData_t fs);
static inline Rover_MetersPerSecondSquared_t RoverImuConfig_Fs2ToMetersPerSecondSquared(const RoverImuConfig_RawData_t fs2);
static inline Rover_MetersPerSecondSquared_t RoverImuConfig_125dpsToRadiansPerSecond(const RoverImuConfig_RawData_t dps);

static stmdev_ctx_t RoverImuConfig_DeviceHandle = {
    .write_reg = RoverImuConfig_Write,
    .read_reg  = RoverImuConfig_Read,
    .mdelay    = Bsp_Delay,
    .handle    = &hspi2,
};

static RoverImuConfig_RawData_t RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_MAX] = {
    [ROVER_IMU_CONFIG_AXIS_X] = 0,
    [ROVER_IMU_CONFIG_AXIS_Y] = 0,
    [ROVER_IMU_CONFIG_AXIS_Z] = 0
};
static RoverImuConfig_RawData_t RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_MAX] = {
    [ROVER_IMU_CONFIG_AXIS_X] = 0,
    [ROVER_IMU_CONFIG_AXIS_Y] = 0,
    [ROVER_IMU_CONFIG_AXIS_Z] = 0
};

/* Onboard registers may not have enough precision to store offset */
static RoverImuConfig_RawData_t RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_MAX] = {
    [ROVER_IMU_CONFIG_AXIS_X] = 0,
    [ROVER_IMU_CONFIG_AXIS_Y] = 0,
    [ROVER_IMU_CONFIG_AXIS_Z] = 0
};
static RoverImuConfig_RawData_t RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_MAX] = {
    [ROVER_IMU_CONFIG_AXIS_X] = 0,
    [ROVER_IMU_CONFIG_AXIS_Y] = 0,
    [ROVER_IMU_CONFIG_AXIS_Z] = 0
};

Rover_Error_t RoverImuConfig_Initialize(void)
{
    RoverImuConfig_Error_t error = ROVER_IMU_CONFIG_ERROR_NONE;

    /* Wait sensor boot time */
    Bsp_Delay(ROVER_IMU_CONFIG_BOOT_TIME);

    /* Check device ID */
    uint8_t whoami = 0U;
    error |= lsm6dsv16x_device_id_get(&RoverImuConfig_DeviceHandle, &whoami);
    if (LSM6DSV16X_ID != whoami)
    {
        error = 1;

        BSP_LOGGER_LOG_ERROR(kRoverImuConfig_LogTag, "Failed to detect IMU");
    }
    else
    {
        BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "IMU detected");

        /* Restore default configuration */
        BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "Resetting IMU");
        lsm6dsv16x_reset_t imu_reset;
        error |= lsm6dsv16x_reset_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_RESTORE_CTRL_REGS);
        error |= lsm6dsv16x_reset_get(&RoverImuConfig_DeviceHandle, &imu_reset);
        while (LSM6DSV16X_READY != imu_reset)
        {
            error |= lsm6dsv16x_reset_get(&RoverImuConfig_DeviceHandle, &imu_reset);
        }
        BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "IMU reset");

        /* Enable Block Data Update */
        error |= lsm6dsv16x_block_data_update_set(&RoverImuConfig_DeviceHandle, PROPERTY_ENABLE);

        /* Set Output Data Rate.
         * Selected data rate have to be equal or greater with respect
         * with MLC data rate.
         */
        error |= lsm6dsv16x_xl_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_ODR_AT_7680Hz);
        error |= lsm6dsv16x_gy_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_ODR_AT_7680Hz);

        /* Set full scale */
        error |= lsm6dsv16x_xl_full_scale_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_2g);
        error |= lsm6dsv16x_gy_full_scale_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_125dps);

        /* Configure filtering chain */
        lsm6dsv16x_filt_settling_mask_t filter_settling_mask;
        filter_settling_mask.drdy   = PROPERTY_ENABLE;
        filter_settling_mask.irq_xl = PROPERTY_ENABLE;
        filter_settling_mask.irq_g  = PROPERTY_ENABLE;
        error                      |= lsm6dsv16x_filt_settling_mask_set(&RoverImuConfig_DeviceHandle, filter_settling_mask);
        error                      |= lsm6dsv16x_filt_gy_lp1_set(&RoverImuConfig_DeviceHandle, PROPERTY_ENABLE);
        error                      |= lsm6dsv16x_filt_gy_lp1_bandwidth_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_GY_ULTRA_LIGHT);
        error                      |= lsm6dsv16x_filt_xl_lp2_set(&RoverImuConfig_DeviceHandle, PROPERTY_ENABLE);
        error                      |= lsm6dsv16x_filt_xl_lp2_bandwidth_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_XL_STRONG);
    }

    if ((ROVER_IMU_CONFIG_ERROR_NONE != error) || (ROVER_ERROR_NONE != RoverImuConfig_Calibrate()))
    {
        error = 1;

        BSP_LOGGER_LOG_ERROR(kRoverImuConfig_LogTag, "Failed to initialize");
    }
    else
    {

        BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "Initialized");
    }

    return RoverImuConfig_ImuToRoverError(error);
}

Rover_Error_t RoverImuConfig_Calibrate(void)
{
    RoverImuConfig_Error_t     error       = ROVER_IMU_CONFIG_ERROR_NONE;
    lsm6dsv16x_fifo_status_t   fifo_status = {
        0U
    };
    uint16_t                   xl_samples                         = 0U;
    uint16_t                   gy_samples                         = 0U;
    int32_t                    xl_data[ROVER_IMU_CONFIG_AXIS_MAX] = {
        0U
    };
    int32_t                    gy_data[ROVER_IMU_CONFIG_AXIS_MAX] = {
        0U
    };
    int32_t                    gravity_offset = (int32_t)(ROVER_IMU_MILLIG_TO_G / RoverImuConfig_FsToMilliG(1));
    lsm6dsv16x_data_rate_t     xl_data_rate;
    lsm6dsv16x_data_rate_t     gy_data_rate;
    lsm6dsv16x_fifo_xl_batch_t xl_fifo_batch;
    lsm6dsv16x_fifo_gy_batch_t gy_fifo_batch;
    lsm6dsv16x_fifo_mode_t     fifo_mode;

    BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "Calibrating");

    /* Save existing data rate */
    error |= lsm6dsv16x_xl_data_rate_get(&RoverImuConfig_DeviceHandle, &xl_data_rate);
    error |= lsm6dsv16x_gy_data_rate_get(&RoverImuConfig_DeviceHandle, &gy_data_rate);
    error |= lsm6dsv16x_fifo_xl_batch_get(&RoverImuConfig_DeviceHandle, &xl_fifo_batch);
    error |= lsm6dsv16x_fifo_gy_batch_get(&RoverImuConfig_DeviceHandle, &gy_fifo_batch);

    /* Save exisitng FIFO mode */
    error |= lsm6dsv16x_fifo_mode_get(&RoverImuConfig_DeviceHandle, &fifo_mode);

    /* Set new data rate for calibration period (1s~2s) to fill FIFO */
    error |= lsm6dsv16x_xl_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_ODR_AT_240Hz);
    error |= lsm6dsv16x_gy_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_ODR_AT_240Hz);
    error |= lsm6dsv16x_fifo_xl_batch_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_XL_BATCHED_AT_240Hz);
    error |= lsm6dsv16x_fifo_gy_batch_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_GY_BATCHED_AT_240Hz);

    Bsp_Delay(ROVER_IMU_CONFIG_CALIBRATION_SETTLE_TIME);

    /* Set FIFO to FIFO mode */
    error |= lsm6dsv16x_fifo_mode_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_FIFO_MODE);

    /* Wait until FIFO is full */
    while (0U == fifo_status.fifo_full)
    {
        error |= lsm6dsv16x_fifo_status_get(&RoverImuConfig_DeviceHandle, &fifo_status);
    }

    /* Read and average FIFO samples */
    for (uint16_t i = 0U; i < fifo_status.fifo_level; i++)
    {
        lsm6dsv16x_fifo_out_raw_t fifo_data;
        error |= lsm6dsv16x_fifo_out_raw_get(&RoverImuConfig_DeviceHandle, &fifo_data);

        RoverImuConfig_RawData_t x = *(RoverImuConfig_RawData_t *)&fifo_data.data[ROVER_IMU_CONFIG_FIFO_DATA_X];
        RoverImuConfig_RawData_t y = *(RoverImuConfig_RawData_t *)&fifo_data.data[ROVER_IMU_CONFIG_FIFO_DATA_Y];
        RoverImuConfig_RawData_t z = *(RoverImuConfig_RawData_t *)&fifo_data.data[ROVER_IMU_CONFIG_FIFO_DATA_Z];

        switch (fifo_data.tag)
        {
        case LSM6DSV16X_XL_NC_TAG:
            xl_data[ROVER_IMU_CONFIG_AXIS_X] += (int32_t)x;
            xl_data[ROVER_IMU_CONFIG_AXIS_Y] += (int32_t)y;
            xl_data[ROVER_IMU_CONFIG_AXIS_Z] += (int32_t)z;
            xl_samples++;
            break;
        case LSM6DSV16X_GY_NC_TAG:
            gy_data[ROVER_IMU_CONFIG_AXIS_X] += (int32_t)x;
            gy_data[ROVER_IMU_CONFIG_AXIS_Y] += (int32_t)y;
            gy_data[ROVER_IMU_CONFIG_AXIS_Z] += (int32_t)z;
            gy_samples++;
            break;
        default:
            break;
        }
    }

    /* Calculate accelerometer offset */
    RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_X] = (int16_t)(xl_data[ROVER_IMU_CONFIG_AXIS_X] / (int32_t)xl_samples);
    RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_Y] = (int16_t)(xl_data[ROVER_IMU_CONFIG_AXIS_Y] / (int32_t)xl_samples);
    RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_Z] = (int16_t)((xl_data[ROVER_IMU_CONFIG_AXIS_Z] / (int32_t)xl_samples) - gravity_offset);

    /* Calculate gyroscope offset */
    RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_X] = (int16_t)(gy_data[ROVER_IMU_CONFIG_AXIS_X] / (int32_t)gy_samples);
    RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_Y] = (int16_t)(gy_data[ROVER_IMU_CONFIG_AXIS_Y] / (int32_t)gy_samples);
    RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_Z] = (int16_t)(gy_data[ROVER_IMU_CONFIG_AXIS_Z] / (int32_t)gy_samples);

    /* Restore data rate */
    error |= lsm6dsv16x_xl_data_rate_set(&RoverImuConfig_DeviceHandle, xl_data_rate);
    error |= lsm6dsv16x_gy_data_rate_set(&RoverImuConfig_DeviceHandle, gy_data_rate);
    error |= lsm6dsv16x_fifo_xl_batch_set(&RoverImuConfig_DeviceHandle, xl_fifo_batch);
    error |= lsm6dsv16x_fifo_gy_batch_set(&RoverImuConfig_DeviceHandle, gy_fifo_batch);

    /* Restore FIFO mode */
    error |= lsm6dsv16x_fifo_mode_set(&RoverImuConfig_DeviceHandle, fifo_mode);

    if (ROVER_IMU_CONFIG_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kRoverImuConfig_LogTag, "Calibration failed");
    }
    else
    {
        BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "Calibrated");
    }

    return RoverImuConfig_ImuToRoverError(error);
}

Rover_Error_t RoverImuConfig_ReadAccelerometer(Rover_AccelerometerReading_t *const reading)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != reading)
    {
        error = RoverImuConfig_ReadAll();

        reading->x = RoverImuConfig_Fs2ToMetersPerSecondSquared(RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_X]);
        reading->y = RoverImuConfig_Fs2ToMetersPerSecondSquared(RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Y]);
        reading->z = RoverImuConfig_Fs2ToMetersPerSecondSquared(RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Z]);
    }

    return error;
}

Rover_Error_t RoverImuConfig_ReadGyroscope(Rover_GyroscopeReading_t *const reading)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != reading)
    {
        error = RoverImuConfig_ReadAll();

        reading->x = RoverImuConfig_125dpsToRadiansPerSecond(RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_X]);
        reading->y = RoverImuConfig_125dpsToRadiansPerSecond(RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Y]);
        reading->z = RoverImuConfig_125dpsToRadiansPerSecond(RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Z]);
    }

    return error;
}

static RoverImuConfig_Error_t RoverImuConfig_Write(void *const handle, const uint8_t imu_register, const uint8_t *const data, const uint16_t size)
{
    RoverImuConfig_Error_t error = ROVER_IMU_CONFIG_ERROR_NONE;

    if ((BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_RESET)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Transmit(handle, &imu_register, 1U, ROVER_IMU_CONFIG_TIMEOUT)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Transmit(handle, data, size, ROVER_IMU_CONFIG_TIMEOUT)) ||
        (BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_SET)))
    {
        error = 1;
    }

    return error;
}

static RoverImuConfig_Error_t RoverImuConfig_Read(void *const handle, const uint8_t imu_register, uint8_t *const data, const uint16_t size)
{
    RoverImuConfig_Error_t error         = ROVER_IMU_CONFIG_ERROR_NONE;
    uint8_t                register_read = imu_register | ROVER_IMU_CONFIG_REGISTER_READ;

    if ((BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_RESET)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Transmit(handle, &register_read, 1U, ROVER_IMU_CONFIG_TIMEOUT)) ||
        (BSP_ERROR_NONE != (Bsp_Error_t)HAL_SPI_Receive(handle, data, size, ROVER_IMU_CONFIG_TIMEOUT)) ||
        (BSP_ERROR_NONE != BspGpio_Write(BSP_GPIO_USER_PIN_IMU_CS, BSP_GPIO_STATE_SET)))
    {
        error = 1;
    }

    return error;
}

static Rover_Error_t RoverImuConfig_ReadAll(void)
{
    RoverImuConfig_Error_t  error = ROVER_IMU_CONFIG_ERROR_NONE;
    lsm6dsv16x_data_ready_t data_ready;

    lsm6dsv16x_flag_data_ready_get(&RoverImuConfig_DeviceHandle, &data_ready);

    if (data_ready.drdy_xl)
    {
        error |= lsm6dsv16x_acceleration_raw_get(&RoverImuConfig_DeviceHandle, RoverImuConfig_RawAccelerometer);

        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_X] -= RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_X];
        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Y] -= RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_Y];
        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Z] -= RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_Z];
    }

    if (data_ready.drdy_gy)
    {
        error |= lsm6dsv16x_angular_rate_raw_get(&RoverImuConfig_DeviceHandle, RoverImuConfig_RawGyroscope);

        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_X] -= RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_X];
        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Y] -= RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_Y];
        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Z] -= RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_Z];
    }

    return RoverImuConfig_ImuToRoverError(error);
}

static inline Rover_Error_t RoverImuConfig_ImuToRoverError(const RoverImuConfig_Error_t error)
{
    Rover_Error_t rover_error = ROVER_ERROR_NONE;

    if (ROVER_IMU_CONFIG_ERROR_NONE != error)
    {
        rover_error = ROVER_ERROR_PERIPHERAL;
    }

    return rover_error;
}

static inline float RoverImuConfig_FsToMilliG(const RoverImuConfig_RawData_t fs)
{
    float                      mg = 0.0f;
    lsm6dsv16x_xl_full_scale_t xl_full_scale;

    (void)lsm6dsv16x_xl_full_scale_get(&RoverImuConfig_DeviceHandle, &xl_full_scale);

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

static inline Rover_MetersPerSecondSquared_t RoverImuConfig_Fs2ToMetersPerSecondSquared(const RoverImuConfig_RawData_t fs2)
{
    return fs2 * ROVER_IMU_CONFIG_FS2_TO_METERS_PER_SECOND_SQUARED;
}

static inline Rover_MetersPerSecondSquared_t RoverImuConfig_125dpsToRadiansPerSecond(const RoverImuConfig_RawData_t dps)
{
    return dps * ROVER_IMU_CONFIG_125DPS_TO_RADIANS_PER_SECOND;
}