#ifndef BSP_TICK_H
#define BSP_TICK_H

#include "bsp.h"

Bsp_Error_t BspTick_Start(void);
Bsp_Millisecond_t BspTick_GetTick(void);
Bsp_Microsecond_t BspTick_GetMicroseconds(void);

#endif /* BSP_TICK_H */