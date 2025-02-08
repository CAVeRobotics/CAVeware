#include "bsp.h"

#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"

extern void SystemClock_Config(void);

void Bsp_Initialize(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
}