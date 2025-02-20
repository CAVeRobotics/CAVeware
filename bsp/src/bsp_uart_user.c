#include "bsp_uart_user.h"

#include "usart.h"

#include "bsp.h"

Bsp_UartHandle_t *const BspUartUser_HandleTable[BSP_UART_USER_MAX] = {
    [BSP_UART_USER_LOG] = &huart2,
};