#include "bsp_uart_user.h"

#include "bsp.h"

extern Bsp_UartHandle_t huart2;

Bsp_UartHandle_t *BspUartUser_HandleTable[BSP_UART_USER_MAX] = {
    [BSP_UART_USER_LOG] = &huart2,
};