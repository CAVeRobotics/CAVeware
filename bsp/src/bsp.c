#include "bsp.h"

#include <stdint.h>

#include "bsp_logger.h"
#include "bsp_logger_user.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"
#include "bsp_user.h"

static const char *kBsp_LogTag = "BSP";

extern void SystemClock_Config(void);

void Bsp_Initialize(void)
{
    HAL_Init();

    SystemClock_Config();

    BspUser_Initialize();

    /* Initialize custom logger */
    BspUart_Start(BSP_UART_USER_LOG);
    BspLoggerUser_RegisterCustomLogger();
    BSP_LOGGER_LOG_DEBUG(kBsp_LogTag, "Initialized");
}

void Bsp_Delay(const Bsp_Millisecond_t delay)
{
    HAL_Delay(delay);
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