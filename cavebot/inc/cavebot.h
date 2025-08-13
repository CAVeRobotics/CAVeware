#ifndef CAVEBOT_H
#define CAVEBOT_H

#include <stdbool.h>

#include "bsp.h"

typedef enum
{
    CAVEBOT_ERROR_NONE,
    CAVEBOT_ERROR_NULL,
    CAVEBOT_ERROR_BSP,
    CAVEBOT_ERROR_PERIPHERAL,
    CAVEBOT_ERROR_MODE,
    CAVEBOT_ERROR_VALUE
} Cavebot_Error_t;

void Cavebot_Initialize(void);
void Cavebot_Task(void);
Cavebot_Error_t Cavebot_BspToCavebotError(const Bsp_Error_t bsp_error);
Cavebot_Error_t Cavebot_Arm(void);
Cavebot_Error_t Cavebot_Disarm(void);
bool Cavebot_IsArmed(void);
Cavebot_Error_t Cavebot_EnableControl(void);
Cavebot_Error_t Cavebot_DisableControl(void);
Cavebot_Error_t Cavebot_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);

#endif /* CAVEBOT_H */