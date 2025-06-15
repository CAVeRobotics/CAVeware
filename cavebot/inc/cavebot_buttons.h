#ifndef CAVEBOT_BUTTONS_H
#define CAVEBOT_BUTTONS_H

#include "bsp.h"

typedef enum
{
    CAVEBOT_BUTTONS_BUTTON_HEADLIGHTS,
    CAVEBOT_BUTTONS_BUTTON_START,
    CAVEBOT_BUTTONS_BUTTON_ENABLE,
    CAVEBOT_BUTTONS_BUTTON_MAX
} CavebotButtons_Button_t;

Bsp_Error_t CavebotButtons_Enable(const CavebotButtons_Button_t button);
Bsp_Error_t CavebotButtons_Disable(const CavebotButtons_Button_t button);

#endif /* CAVEBOT_BUTTONS_H */