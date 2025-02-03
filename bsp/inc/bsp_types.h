#ifndef BSP_TYPES_H
#define BSP_TYPES_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef double BspTypes_Percent_t;
typedef uint32_t BspTypes_Microsecond_t;

typedef TIM_HandleTypeDef BspTypes_TimerHandle_t;

typedef enum
{
    BSP_TYPES_ERROR_NONE = HAL_OK,
    BSP_TYPES_ERROR_HAL = HAL_ERROR,
    BSP_TYPES_ERROR_BUSY = HAL_BUSY,
    BSP_TYPES_ERROR_TIMEOUT = HAL_TIMEOUT,
    BSP_TYPES_ERROR_PERIPHERAL = 0x04U,
    BSP_TYPES_ERROR_VALUE = 0x05U,
    BSP_TYPES_ERROR_NULL = 0x06U,
} BspTypes_Error_t;

typedef enum
{
    BSP_TYPES_TIMER_CHANNEL_1 = TIM_CHANNEL_1,
    BSP_TYPES_TIMER_CHANNEL_2 = TIM_CHANNEL_2,
    BSP_TYPES_TIMER_CHANNEL_3 = TIM_CHANNEL_3,
    BSP_TYPES_TIMER_CHANNEL_4 = TIM_CHANNEL_4
} BspTypes_TimerChannel_t;

#endif /* BSP_TYPES_H */