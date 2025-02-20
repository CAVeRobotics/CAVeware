#ifndef BSP_UART_USER_H
#define BSP_UART_USER_H

#include "bsp.h"

typedef enum
{
    BSP_UART_USER_LOG,
    BSP_UART_USER_MAX
} BspUartUser_Uart_t;

extern Bsp_UartHandle_t *const BspUartUser_HandleTable[BSP_UART_USER_MAX];

#endif /* BSP_UART_USER_H */