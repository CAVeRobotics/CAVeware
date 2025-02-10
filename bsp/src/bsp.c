#include "bsp.h"

#include <stdint.h>

#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"

#include "bsp_logger.h"
#include "bsp_logger_user.h"

static const char *kBsp_LogTag = "BSP";

extern void SystemClock_Config(void);

void Bsp_Initialize(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    BspLoggerUser_RegisterCustomLogger();
    BSP_LOGGER_LOG_DEBUG(kBsp_LogTag, "Initialized");
}

Bsp_Millisecond_t Bsp_GetTick(void)
{
    return HAL_GetTick();
}