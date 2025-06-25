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
    ROVER_IMU_CONFIG_QUATERION_AXIS_W,
    ROVER_IMU_CONFIG_QUATERION_AXIS_X,
    ROVER_IMU_CONFIG_QUATERION_AXIS_Y,
    ROVER_IMU_CONFIG_QUATERION_AXIS_Z,
    ROVER_IMU_CONFIG_QUATERION_AXIS_MAX
} RoverImuConfig_QuaterionAxis_t;

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
static Rover_Error_t RoverImuConfig_ReadFifo(void);
static inline Rover_Error_t RoverImuConfig_ImuToRoverError(const RoverImuConfig_Error_t error);
static inline float RoverImuConfig_FsToMilliG(const RoverImuConfig_RawData_t fs);
static inline Rover_MetersPerSecondSquared_t RoverImuConfig_Fs2ToMetersPerSecondSquared(const RoverImuConfig_RawData_t fs2);
static inline Rover_MetersPerSecondSquared_t RoverImuConfig_125dpsToRadiansPerSecond(const RoverImuConfig_RawData_t dps);

/* Third-party code to handle SFLP quaterion conversion
   Non-compliant, DO NOT MODIFY
   Source: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c
 */
static float_t npy_half_to_float(uint16_t h);
static void sflp2q(float_t quat[4], const uint16_t sflp[3]);

static stmdev_ctx_t RoverImuConfig_DeviceHandle = {
    .write_reg = RoverImuConfig_Write,
    .read_reg  = RoverImuConfig_Read,
    .mdelay    = Bsp_Delay,
    .handle    = &hspi1,
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
static double                   RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_MAX] = {
    [ROVER_IMU_CONFIG_QUATERION_AXIS_W] = 0.0,
    [ROVER_IMU_CONFIG_QUATERION_AXIS_X] = 0.0,
    [ROVER_IMU_CONFIG_QUATERION_AXIS_Y] = 0.0,
    [ROVER_IMU_CONFIG_QUATERION_AXIS_Z] = 0.0
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

        /* Enable game rotation vector */
        lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
        fifo_sflp.game_rotation = 1;
        fifo_sflp.gravity       = 0;
        fifo_sflp.gbias         = 0;
        error                  |= lsm6dsv16x_fifo_sflp_batch_set(&RoverImuConfig_DeviceHandle, fifo_sflp);

        /* Set FIFO mode to Continuous mode */
        error |= lsm6dsv16x_fifo_mode_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_STREAM_MODE);

        /* Set Game Vector Output Data Rate */
        error |= lsm6dsv16x_sflp_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_SFLP_480Hz);
        error |= lsm6dsv16x_sflp_game_rotation_set(&RoverImuConfig_DeviceHandle, PROPERTY_ENABLE);
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
    RoverImuConfig_Error_t      error       = ROVER_IMU_CONFIG_ERROR_NONE;
    lsm6dsv16x_fifo_status_t    fifo_status = {
        0U
    };
    uint16_t                    xl_samples                         = 0U;
    uint16_t                    gy_samples                         = 0U;
    int32_t                     xl_data[ROVER_IMU_CONFIG_AXIS_MAX] = {
        0U
    };
    int32_t                     gy_data[ROVER_IMU_CONFIG_AXIS_MAX] = {
        0U
    };
    int32_t                     gravity_offset = (int32_t)(ROVER_IMU_MILLIG_TO_G / RoverImuConfig_FsToMilliG(1));
    lsm6dsv16x_data_rate_t      xl_data_rate;
    lsm6dsv16x_data_rate_t      gy_data_rate;
    lsm6dsv16x_fifo_xl_batch_t  xl_fifo_batch;
    lsm6dsv16x_fifo_gy_batch_t  gy_fifo_batch;
    lsm6dsv16x_fifo_mode_t      fifo_mode;
    lsm6dsv16x_fifo_sflp_raw_t  fifo_sflp;
    lsm6dsv16x_sflp_data_rate_t sflp_data_rate;
    uint8_t                     game_rotation_enabled;

    BSP_LOGGER_LOG_DEBUG(kRoverImuConfig_LogTag, "Calibrating");

    /* Save existing data rate */
    error |= lsm6dsv16x_xl_data_rate_get(&RoverImuConfig_DeviceHandle, &xl_data_rate);
    error |= lsm6dsv16x_gy_data_rate_get(&RoverImuConfig_DeviceHandle, &gy_data_rate);
    error |= lsm6dsv16x_fifo_xl_batch_get(&RoverImuConfig_DeviceHandle, &xl_fifo_batch);
    error |= lsm6dsv16x_fifo_gy_batch_get(&RoverImuConfig_DeviceHandle, &gy_fifo_batch);

    /* Save exisitng FIFO mode */
    error |= lsm6dsv16x_fifo_mode_get(&RoverImuConfig_DeviceHandle, &fifo_mode);

    /* Save existing SFLP settings */
    error |= lsm6dsv16x_fifo_sflp_batch_get(&RoverImuConfig_DeviceHandle, &fifo_sflp);
    error |= lsm6dsv16x_sflp_data_rate_get(&RoverImuConfig_DeviceHandle, &sflp_data_rate);
    error |= lsm6dsv16x_sflp_game_rotation_get(&RoverImuConfig_DeviceHandle, &game_rotation_enabled);

    /* Set new data rate for calibration period (1s~2s) to fill FIFO */
    error |= lsm6dsv16x_xl_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_ODR_AT_240Hz);
    error |= lsm6dsv16x_gy_data_rate_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_ODR_AT_240Hz);
    error |= lsm6dsv16x_fifo_xl_batch_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_XL_BATCHED_AT_240Hz);
    error |= lsm6dsv16x_fifo_gy_batch_set(&RoverImuConfig_DeviceHandle, LSM6DSV16X_GY_BATCHED_AT_240Hz);

    /* Disable SFLP */
    lsm6dsv16x_fifo_sflp_raw_t fifo_sflp_disabled;
    fifo_sflp_disabled.game_rotation = 0;
    fifo_sflp_disabled.gravity       = 0;
    fifo_sflp_disabled.gbias         = 0;
    error                           |= lsm6dsv16x_fifo_sflp_batch_set(&RoverImuConfig_DeviceHandle, fifo_sflp_disabled);
    error                           |= lsm6dsv16x_sflp_game_rotation_set(&RoverImuConfig_DeviceHandle, PROPERTY_DISABLE);

    Bsp_Delay(ROVER_IMU_CONFIG_CALIBRATION_SETTLE_TIME);

    /* Flush FIFO */
    error |= lsm6dsv16x_fifo_status_get(&RoverImuConfig_DeviceHandle, &fifo_status);
    for (uint16_t i = 0U; i < fifo_status.fifo_level; i++)
    {
        lsm6dsv16x_fifo_out_raw_t fifo_data;
        error |= lsm6dsv16x_fifo_out_raw_get(&RoverImuConfig_DeviceHandle, &fifo_data);
    }

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

    /* Restore SFLP settings */
    error |= lsm6dsv16x_fifo_sflp_batch_set(&RoverImuConfig_DeviceHandle, fifo_sflp);
    error |= lsm6dsv16x_sflp_data_rate_set(&RoverImuConfig_DeviceHandle, sflp_data_rate);
    error |= lsm6dsv16x_sflp_game_rotation_set(&RoverImuConfig_DeviceHandle, game_rotation_enabled);

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

Rover_Error_t RoverImuConfig_ReadQuaterion(Rover_Quaternion_t *const quaternion)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != quaternion)
    {
        error = RoverImuConfig_ReadFifo();

        quaternion->w = RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_W];
        quaternion->x = RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_X];
        quaternion->y = RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_Y];
        quaternion->z = RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_Z];
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

    error |= lsm6dsv16x_flag_data_ready_get(&RoverImuConfig_DeviceHandle, &data_ready);

    if (data_ready.drdy_xl)
    {
        error |= lsm6dsv16x_acceleration_raw_get(&RoverImuConfig_DeviceHandle, RoverImuConfig_RawAccelerometer);

        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_X] -= RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_X];
        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Y] -= RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_Y];
        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Z] -= RoverImuConfig_AccelerometerOffset[ROVER_IMU_CONFIG_AXIS_Z];

        /* Correct for IMU orientation in rover */
        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_X] *= -1;
        RoverImuConfig_RawAccelerometer[ROVER_IMU_CONFIG_AXIS_Y] *= -1;
    }

    if (data_ready.drdy_gy)
    {
        error |= lsm6dsv16x_angular_rate_raw_get(&RoverImuConfig_DeviceHandle, RoverImuConfig_RawGyroscope);

        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_X] -= RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_X];
        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Y] -= RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_Y];
        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Z] -= RoverImuConfig_GyroscopeOffset[ROVER_IMU_CONFIG_AXIS_Z];

        /* Correct for IMU orientation in rover */
        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_X] *= -1;
        RoverImuConfig_RawGyroscope[ROVER_IMU_CONFIG_AXIS_Y] *= -1;
    }

    return RoverImuConfig_ImuToRoverError(error);
}

static Rover_Error_t RoverImuConfig_ReadFifo(void)
{
    lsm6dsv16x_fifo_status_t fifo_status;
    float_t                  quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_MAX] = {
        0U
    };
    float_t                  quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_MAX] = {
        0U
    };
    uint16_t                 game_rotation_vector_samples = 0U;
    RoverImuConfig_Error_t   error                        = lsm6dsv16x_fifo_status_get(&RoverImuConfig_DeviceHandle, &fifo_status);

    for (uint16_t i = 0U; i < fifo_status.fifo_level; i++)
    {
        lsm6dsv16x_fifo_out_raw_t fifo_data;
        error |= lsm6dsv16x_fifo_out_raw_get(&RoverImuConfig_DeviceHandle, &fifo_data);

        switch (fifo_data.tag)
        {
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
            sflp2q(quaternion, (uint16_t *)&fifo_data.data[0]);
            quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_W] += quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_W];
            quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_X] += quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_X];
            quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_Y] += quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_Y];
            quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_Z] += quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_Z];
            game_rotation_vector_samples++;
        default:
            break;
        }
    }

    RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_W] = (double)quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_W] / game_rotation_vector_samples;
    RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_X] = (double)quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_X] / game_rotation_vector_samples;
    RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_Y] = (double)quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_Y] / game_rotation_vector_samples;
    RoverImuConfig_Quaternion[ROVER_IMU_CONFIG_QUATERION_AXIS_Z] = (double)quaternion_averaged[ROVER_IMU_CONFIG_QUATERION_AXIS_Z] / game_rotation_vector_samples;

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
    }
    conv;
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
        sumsq    = 1.0f;
    }

    quat[3] = sqrtf(1.0f - sumsq);
}