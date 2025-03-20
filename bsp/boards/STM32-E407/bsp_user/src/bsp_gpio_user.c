#include "bsp_gpio_user.h"

#include "main.h"

#include "bsp.h"

Bsp_Gpio_t BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_MAX] = {
    [BSP_GPIO_USER_PIN_HEADLIGHTS_0] = {
        .gpio_port = GPIOG,
        .gpio_pin  = GPIO_PIN_4,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_HEADLIGHTS_1] = {
        .gpio_port = GPIOG,
        .gpio_pin  = GPIO_PIN_5,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_HEADLIGHTS_2] = {
        .gpio_port = GPIOG,
        .gpio_pin  = GPIO_PIN_6,
        .mode      = BSP_GPIO_MODE_OUTPUT,
        .callback  = NULL,
        .debounce  = 0U,
        .previous  = 0U,
    },
    [BSP_GPIO_USER_PIN_HEADLIGHTS_ENABLE] = {
        .gpio_port = HEADLIGHTS_ENABLE_GPIO_Port,
        .gpio_pin  = HEADLIGHTS_ENABLE_Pin,
        .mode      = BSP_GPIO_MODE_INPUT,
        .callback  = NULL,
        .debounce  = 50000U,
        .previous  = 0U,
    },
};

Bsp_Gpio_t *BspGpioUser_GetGpioHandle(const Bsp_GpioPin_t exti_pin)
{
    Bsp_Gpio_t *gpio = NULL;

    switch (exti_pin)
    {
    case HEADLIGHTS_ENABLE_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_HEADLIGHTS_ENABLE];
    default:
        break;
    }

    return gpio;
}