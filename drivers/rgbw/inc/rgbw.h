#ifndef RGBW_H
#define RGBW_H

#include "bsp.h"
#include "bsp_gpio_user.h"

typedef enum
{
    RGBW_CHANNEL_RED,
    RGBW_CHANNEL_GREEN,
    RGBW_CHANNEL_BLUE,
    RGBW_CHANNEL_WHITE,
    RGBW_CHANNEL_MAX
} Rgbw_Channel_t;

typedef enum
{
    RGBW_COLOR_BLACK,
    RGBW_COLOR_RED,
    RGBW_COLOR_GREEN,
    RGBW_COLOR_BLUE,
    RGBW_COLOR_WHITE,
    RGBW_COLOR_YELLOW,
    RGBW_COLOR_MAGENTA,
    RGBW_COLOR_CYAN,
    RGBW_COLOR_MAX
} Rgbw_Color_t;

typedef struct
{
    BspGpioUser_Pin_t pins[RGBW_CHANNEL_MAX];
} Rgbw_Handle_t;

Bsp_Error_t Rgbw_SetColor(const Rgbw_Handle_t *const handle, const Rgbw_Color_t color);

#endif /* RGBW_H */