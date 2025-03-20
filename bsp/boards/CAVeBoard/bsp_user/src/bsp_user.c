#include "bsp_user.h"

#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

void BspUser_Initialize(void)
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM12_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
}