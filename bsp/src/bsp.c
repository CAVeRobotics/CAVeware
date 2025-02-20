#include "bsp.h"

#include <stdint.h>

#include "gpio.h"
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
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM10_Init();

    BspLoggerUser_RegisterCustomLogger();
    BSP_LOGGER_LOG_DEBUG(kBsp_LogTag, "Initialized");
}

double Bsp_Map(const double value, const double in_min, const double in_max, const double out_min, const double out_max)
{
    double capped_value = value;

    if (value < in_min)
    {
        capped_value = in_min;
    }
    if (value > in_max)
    {
        capped_value = in_max;
    }

    return (capped_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}