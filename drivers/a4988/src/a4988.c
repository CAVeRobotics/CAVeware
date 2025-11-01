#include "a4988.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "bsp_gpio.h"
#include "bsp_timer.h"

#define A4988_HALF_PULSES_PER_PULSE 2U

static void A4988_OnStep(void *arg);

Bsp_Error_t A4988_Initialize(A9488_Context_t *const context)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != context)
    {
        Bsp_Callback_t callback = {
            .function = A4988_OnStep,
            .arg      = context,
        };

        error = BspGpio_Write(context->step, BSP_GPIO_STATE_RESET);

        if (BSP_ERROR_NONE == error)
        {
            error = BspTimer_RegisterPeriodElapsedCallback(context->timer, &callback);
        }
    }

    return error;
}

Bsp_Error_t A4988_Move(A9488_Context_t *const context, const int32_t steps)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL != context)
    {
        context->half_pulses = A4988_HALF_PULSES_PER_PULSE * abs(steps) - 1; /* Subtract 1 for first half pulse executed here */

        (void)BspTimer_Reset(context->timer);

        /* Set direction */
        if (steps < 0)
        {
            error = BspGpio_Write(context->direction, BSP_GPIO_STATE_SET); /* High = Clockwise = Negative */
        }
        else
        {
            error = BspGpio_Write(context->direction, BSP_GPIO_STATE_RESET); /* Low = Counter Clockwise = Positive */
        }

        /* First half pulse (i.e. first step) */
        if (BSP_ERROR_NONE == error)
        {
            error = BspGpio_Write(context->step, BSP_GPIO_STATE_SET);
        }

        /* Start interrupt to run subsequent steps */
        if (BSP_ERROR_NONE == error)
        {
            error = BspTimer_Start(context->timer);
        }
    }

    return error;
}

bool A4988_IsMoving(const A9488_Context_t *const context)
{
    bool moving = false;

    if (A4988_GetRemainingSteps(context) > 0)
    {
        moving = true;
    }

    return moving;
}

int32_t A4988_GetRemainingSteps(const A9488_Context_t *const context)
{
    int32_t steps = 0U;

    if (NULL != context)
    {
        steps = (int32_t)(context->half_pulses / A4988_HALF_PULSES_PER_PULSE);
    }

    return steps;
}

static void A4988_OnStep(void *arg)
{
    A9488_Context_t *context = (A9488_Context_t *)arg;

    if (NULL != context)
    {
        BspGpio_Toggle(context->step);
        context->half_pulses--;

        if (0 == context->half_pulses)
        {
            (void)BspTimer_Stop(context->timer);
        }
    }
}