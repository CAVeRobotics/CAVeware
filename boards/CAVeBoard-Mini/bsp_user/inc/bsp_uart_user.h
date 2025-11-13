#ifndef BSP_UART_USER_H
#define BSP_UART_USER_H

#include "bsp.h"

typedef enum
{
    BSP_UART_USER_0,
    BSP_UART_USER_1,
    BSP_UART_USER_MAX
} BspUartUser_Uart_t;

extern Bsp_Uart_t BspUartUser_HandleTable[BSP_UART_USER_MAX];

#endif /* BSP_UART_USER_H */