#ifndef BSP_UART_USER_H
#define BSP_UART_USER_H

#include "bsp.h"

typedef enum
{
    BSP_UART_USER_0,
    BSP_UART_USER_1,
    BSP_UART_USER_2,
    BSP_UART_USER_MAX
} BspUartUser_Uart_t;

extern Bsp_Uart_t BspUartUser_HandleTable[BSP_UART_USER_MAX];

Bsp_Uart_t *BspUartUser_GetUart(const Bsp_UartHandle_t *const uart_handle);

#endif /* BSP_UART_USER_H */