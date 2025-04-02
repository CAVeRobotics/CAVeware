#include "bsp_gpio_user.h"

#include "main.h"

#include "bsp.h"

Bsp_Gpio_t BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MAX] = {
    [BSP_GPIO_USER_PIN_HEADLIGHTS_0] = {
        .gpio_port = LED_POD_0_GPIO_Port,
        .gpio_pin  = LED_POD_0_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_HEADLIGHTS_1] = {
        .gpio_port = LED_POD_1_GPIO_Port,
        .gpio_pin  = LED_POD_1_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_HEADLIGHTS_2] = {
        .gpio_port = LED_POD_2_GPIO_Port,
        .gpio_pin  = LED_POD_2_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_HEADLIGHTS_ENABLE] = {
        .gpio_port = HEADLIGHTS_ENABLE_BUTTON_GPIO_Port,
        .gpio_pin  = HEADLIGHTS_ENABLE_BUTTON_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = NULL,
        .debounce  = 50000U, /* TODO SD-130 tune for normal button press */
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_IMU_CS] = {
        .gpio_port = IMU_CS_GPIO_Port,
        .gpio_pin  = IMU_CS_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_IMU_STATUS] = {
        .gpio_port = IMU_STATUS_LED_GPIO_Port,
        .gpio_pin  = IMU_STATUS_LED_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_COMMS_STATUS] = {
        .gpio_port = JETSON_COMMS_LED_GPIO_Port,
        .gpio_pin  = JETSON_COMMS_LED_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_START] = {
        .gpio_port = START_BUTTON_GPIO_Port,
        .gpio_pin  = START_BUTTON_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = NULL,
        .debounce  = 50000U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_ENABLE] = {
        .gpio_port = MOTOR_ENABLE_BUTTON_GPIO_Port,
        .gpio_pin  = MOTOR_ENABLE_BUTTON_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = NULL,
        .debounce  = 50000U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_MOTOR_SLEEP] = {
        .gpio_port = MOTOR_SLEEP_GPIO_Port,
        .gpio_pin  = MOTOR_SLEEP_Pin,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U
    },
};

Bsp_Gpio_t *BspGpioUser_GetGpioHandle(const Bsp_GpioPin_t exti_pin)
{
    Bsp_Gpio_t *gpio = NULL;

    switch (exti_pin)
    {
    case HEADLIGHTS_ENABLE_BUTTON_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_HEADLIGHTS_ENABLE];
        break;
    case START_BUTTON_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_START];
        break;
    case MOTOR_ENABLE_BUTTON_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_ENABLE];
        break;
    default:
        break;
    }

    return gpio;
}