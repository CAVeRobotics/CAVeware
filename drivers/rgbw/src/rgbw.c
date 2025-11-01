#include "rgbw.h"

#include "bsp_gpio.h"

static const Bsp_GpioState_t Rgbw_Colors[RGBW_COLOR_MAX][RGBW_CHANNEL_MAX] = {
    [RGBW_COLOR_BLACK] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
    [RGBW_COLOR_RED] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
    [RGBW_COLOR_GREEN] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
    [RGBW_COLOR_BLUE] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
    [RGBW_COLOR_WHITE] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_SET,
    },
    [RGBW_COLOR_YELLOW] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
    [RGBW_COLOR_MAGENTA] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
    [RGBW_COLOR_CYAN] = {
        [RGBW_CHANNEL_RED]   = BSP_GPIO_STATE_RESET,
        [RGBW_CHANNEL_GREEN] = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_BLUE]  = BSP_GPIO_STATE_SET,
        [RGBW_CHANNEL_WHITE] = BSP_GPIO_STATE_RESET,
    },
};

Bsp_Error_t Rgbw_SetColor(const Rgbw_Handle_t *const handle, const Rgbw_Color_t color)
{
    Bsp_Error_t error = BSP_ERROR_VALUE;

    if (NULL == handle)
    {
        error = BSP_ERROR_NULL;
    }
    else if (color < RGBW_COLOR_MAX)
    {
        for (Rgbw_Channel_t channel = RGBW_CHANNEL_RED; channel < RGBW_CHANNEL_MAX; channel++)
        {
            error = BspGpio_Write(handle->pins[channel], Rgbw_Colors[color][channel]);

            if (BSP_ERROR_NONE != error)
            {
                break;
            }
        }
    }

    return error;
}