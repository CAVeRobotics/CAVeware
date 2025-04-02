#ifndef CAVEMAN_BUTTONS_H
#define CAVEMAN_BUTTONS_H

#include "bsp.h"

typedef enum
{
    CAVEMAN_BUTTONS_BUTTON_HEADLIGHTS,
    CAVEMAN_BUTTONS_BUTTON_START,
    CAVEMAN_BUTTONS_BUTTON_ENABLE,
    CAVEMAN_BUTTONS_BUTTON_MAX
} CavemanButtons_Button_t;

Bsp_Error_t CavemanButtons_Enable(const CavemanButtons_Button_t button);
Bsp_Error_t CavemanButtons_Disable(const CavemanButtons_Button_t button);

#endif /* CAVEMAN_BUTTONS_H */