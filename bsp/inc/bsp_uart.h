#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>

#include "bsp.h"
#include "bsp_uart_user.h"

Bsp_Error_t BspUart_Start(const BspUartUser_Uart_t uart);
Bsp_Error_t BspUart_Transmit(const BspUartUser_Uart_t uart, const uint8_t *const data, const size_t size);
Bsp_Error_t BspUart_Receive(const BspUartUser_Uart_t uart, uint8_t *const data, const size_t size, size_t *const bytes_read);

#endif /* BSP_UART_H */