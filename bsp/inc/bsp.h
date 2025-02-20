#ifndef BSP_H
#define BSP_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_UNUSED(x) (void)(x)

typedef double   Bsp_Percent_t;
typedef double   Bsp_Radian_t;
typedef double   Bsp_RadiansPerSecond_t;
typedef uint32_t Bsp_Millisecond_t;
typedef uint64_t Bsp_Microsecond_t;

typedef TIM_HandleTypeDef  Bsp_TimerHandle_t;
typedef UART_HandleTypeDef Bsp_UartHandle_t;

typedef enum
{
    BSP_ERROR_NONE = HAL_OK,
    BSP_ERROR_HAL = HAL_ERROR,
    BSP_ERROR_BUSY = HAL_BUSY,
    BSP_ERROR_TIMEOUT = HAL_TIMEOUT,
    BSP_ERROR_PERIPHERAL = 0x04U,
    BSP_ERROR_VALUE = 0x05U,
    BSP_ERROR_NULL = 0x06U,
} Bsp_Error_t;

typedef enum
{
    BSP_TIMER_CHANNEL_1 = TIM_CHANNEL_1,
    BSP_TIMER_CHANNEL_2 = TIM_CHANNEL_2,
    BSP_TIMER_CHANNEL_3 = TIM_CHANNEL_3,
    BSP_TIMER_CHANNEL_4 = TIM_CHANNEL_4,
    BSP_TIMER_CHANNEL_ALL = TIM_CHANNEL_ALL
} Bsp_TimerChannel_t;

void Bsp_Initialize(void);
double Bsp_Map(const double value, const double in_min, const double in_max, const double out_min, const double out_max);

#endif /* BSP_H */