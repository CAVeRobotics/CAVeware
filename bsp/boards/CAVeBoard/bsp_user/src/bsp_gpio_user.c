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
        .debounce  = 100000U,
        .previous  = 0U,
    },
};

Bsp_Gpio_t *BspGpioUser_GetGpioHandle(const Bsp_GpioPin_t exti_pin)
{
    Bsp_Gpio_t *gpio = NULL;

    switch (exti_pin)
    {
    case HEADLIGHTS_ENABLE_BUTTON_Pin:
        gpio = &BspGpioUser_HandleTable[BSP_GPIO_USER_PIN_HEADLIGHTS_ENABLE];
    default:
        break;
    }

    return gpio;
}