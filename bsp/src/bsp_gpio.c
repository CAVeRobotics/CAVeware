#include "bsp_gpio.h"

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_gpio_user.h"
#include "bsp_tick.h"

Bsp_Error_t BspGpio_Read(const BspGpioUser_Pin_t pin, Bsp_GpioState_t *const state)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if ((pin >= BSP_GPIO_USER_PIN_MAX) || (BSP_GPIO_MODE_INPUT != BspGpioUser_HandleTable[pin].mode))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        *state = (Bsp_GpioState_t)HAL_GPIO_ReadPin(BspGpioUser_HandleTable[pin].gpio_port, BspGpioUser_HandleTable[pin].gpio_pin);
    }

    return error;
}

Bsp_Error_t BspGpio_Write(const BspGpioUser_Pin_t pin, const Bsp_GpioState_t state)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if ((pin >= BSP_GPIO_USER_PIN_MAX) || (BSP_GPIO_MODE_OUTPUT != BspGpioUser_HandleTable[pin].mode))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        HAL_GPIO_WritePin(BspGpioUser_HandleTable[pin].gpio_port, BspGpioUser_HandleTable[pin].gpio_pin, (GPIO_PinState)state);
    }

    return error;
}

Bsp_Error_t BspGpio_Toggle(const BspGpioUser_Pin_t pin)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if ((pin >= BSP_GPIO_USER_PIN_MAX) || (BSP_GPIO_MODE_OUTPUT != BspGpioUser_HandleTable[pin].mode))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        HAL_GPIO_TogglePin(BspGpioUser_HandleTable[pin].gpio_port, BspGpioUser_HandleTable[pin].gpio_pin);
    }

    return error;
}

Bsp_Error_t BspGpio_RegisterCallback(const BspGpioUser_Pin_t pin, void (*callback)(const Bsp_GpioPin_t pin))
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if ((pin >= BSP_GPIO_USER_PIN_MAX) || (BSP_GPIO_MODE_INPUT != BspGpioUser_HandleTable[pin].mode))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        BspGpioUser_HandleTable[pin].callback = callback;
        BspGpioUser_HandleTable[pin].previous = 0U;
    }

    return error;
}

void HAL_GPIO_EXTI_Callback(Bsp_GpioPin_t pin)
{
    Bsp_Gpio_t *      gpio = BspGpioUser_GetGpioHandle(pin);
    Bsp_Microsecond_t tick = BspTick_GetMicroseconds();

    if ((NULL != gpio) && (NULL != gpio->callback) && ((tick - gpio->previous) >= gpio->debounce))
    {
        gpio->callback(pin);
        gpio->previous = tick;
    }
}