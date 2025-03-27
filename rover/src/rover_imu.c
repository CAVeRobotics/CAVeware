#include "rover_imu.h"

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_logger.h"

#include "rover_imu_config.h"

static const char * kRoverImu_LogTag = "ROVER IMU";

Rover_Error_t RoverImu_Initialize(void)
{
    (void)BspGpio_Write(BSP_GPIO_USER_PIN_IMU_STATUS, BSP_GPIO_STATE_RESET);

    Rover_Error_t error = RoverImuConfig_Initialize();

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kRoverImu_LogTag, "Failed to initialize");
    }
    else
    {
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_IMU_STATUS, BSP_GPIO_STATE_SET);
        BSP_LOGGER_LOG_INFO(kRoverImu_LogTag, "Initialized");
    }

    return error;
}

Rover_Error_t RoverImu_ReadAccelerometer(Rover_AccelerometerReading_t *const reading)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != reading)
    {
        error = RoverImuConfig_ReadAccelerometer(reading);
    }

    return error;
}

Rover_Error_t RoverImu_ReadGyroscope(Rover_GyroscopeReading_t *const reading)
{
    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != reading)
    {
        error = RoverImuConfig_ReadGyroscope(reading);
    }

    return error;
}