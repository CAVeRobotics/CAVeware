#include "bsp_user.h"

#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

void BspUser_Initialize(void)
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM6_Init();
}