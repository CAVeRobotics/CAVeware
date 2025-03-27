#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "bsp.h"
#include "bsp_gpio_user.h"

Bsp_Error_t BspGpio_Read(const BspGpioUser_Pin_t pin, Bsp_GpioState_t *const state);
Bsp_Error_t BspGpio_Write(const BspGpioUser_Pin_t pin, const Bsp_GpioState_t state);
Bsp_Error_t BspGpio_Toggle(const BspGpioUser_Pin_t pin);
Bsp_Error_t BspGpio_RegisterCallback(const BspGpioUser_Pin_t pin, void (*callback)(const Bsp_GpioPin_t pin));

#endif /* BSP_GPIO_H */