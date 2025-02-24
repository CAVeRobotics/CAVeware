#ifndef BSP_UART_USER_H
#define BSP_UART_USER_H

#include "bsp.h"

typedef enum
{
    BSP_UART_USER_LOG,
    BSP_UART_USER_COMMS,
    BSP_UART_USER_MAX
} BspUartUser_Uart_t;

extern Bsp_Uart_t BspUartUser_HandleTable[BSP_UART_USER_MAX];

Bsp_Uart_t* BspUartUser_GetUart(const Bsp_UartHandle_t *const uart_handle);

#endif /* BSP_UART_USER_H */