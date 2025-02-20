#include "bsp_uart.h"

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_uart_user.h"

Bsp_Error_t BspUart_Transmit(const BspUartUser_Uart_t uart, const uint8_t *const data, const size_t size)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == data)
    {
    }
    else if (uart >= BSP_UART_USER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        error = (Bsp_Error_t)HAL_UART_Transmit(BspUartUser_HandleTable[uart], data, size, HAL_MAX_DELAY);
    }

    return error;
}

Bsp_Error_t BspUart_Receive(const BspUartUser_Uart_t uart, uint8_t *const data, const size_t size)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == data)
    {

    }
    else if (uart >= BSP_UART_USER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        error = (Bsp_Error_t)HAL_UART_Receive(BspUartUser_HandleTable[uart], data, size, HAL_MAX_DELAY);
    }

    return error;
}