#ifndef BSP_TICK_H
#define BSP_TICK_H

#include "bsp.h"

#define BSP_TICK_MICROSECONDS_PER_SECOND (double)1e6

Bsp_Error_t BspTick_Start(void);
Bsp_Millisecond_t BspTick_GetTick(void);
Bsp_Microsecond_t BspTick_GetMicroseconds(void);
Bsp_Second_t BspTick_GetElapsedMicroseconds(const Bsp_Microsecond_t start, const Bsp_Microsecond_t end);

#endif /* BSP_TICK_H */