#ifndef A4988_H
#define A4988_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_gpio_user.h"
#include "bsp_timer_user.h"

typedef struct
{
    BspTimerUser_Timer_t timer;
    BspGpioUser_Pin_t step;
    BspGpioUser_Pin_t direction;
    volatile uint32_t half_pulses;
} A9488_Context_t;

Bsp_Error_t A4988_Initialize(A9488_Context_t *const context);
Bsp_Error_t A4988_Move(A9488_Context_t *const context, const int32_t steps);
bool A4988_IsMoving(const A9488_Context_t *const context);
int32_t A4988_GetRemainingSteps(const A9488_Context_t *const context);

#endif /* A4988_H */