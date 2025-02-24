#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_uart_user.h"

extern void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);

static void BspUart_TxCallback(Bsp_UartHandle_t *uart_handle);
static void BspUart_RxCallback(Bsp_UartHandle_t *uart_handle);

Bsp_Error_t BspUart_Start(const BspUartUser_Uart_t uart, void (*tx_callback)(Bsp_Uart_t *const uart), void (*rx_callback)(Bsp_Uart_t *const uart))
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (uart < BSP_UART_USER_MAX)
    {
        BspUartUser_HandleTable[uart].uart_handle->TxCpltCallback = BspUart_TxCallback;
        BspUartUser_HandleTable[uart].uart_handle->RxCpltCallback = BspUart_RxCallback;

        if (NULL != tx_callback)
        {
            BspUartUser_HandleTable[uart].tx_callback = tx_callback;
        }

        if (NULL != rx_callback)
        {
            BspUartUser_HandleTable[uart].rx_callback = rx_callback;
        }

        /* TODO SD-234 error and abort callbacks */

        HAL_UART_MspInit(BspUartUser_HandleTable[uart].uart_handle);
    }

    return error;
}

Bsp_Error_t BspUart_Transmit(const BspUartUser_Uart_t uart, const uint8_t *const data, const size_t size)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (NULL == data)
    {
        error = BSP_ERROR_NULL;
    }
    else if (uart >= BSP_UART_USER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        /* Critical section start */
        BspUartUser_HandleTable[uart].tx_lock = true;

        uint8_t  unlocked       = BspUartUser_HandleTable[uart].tx_unlocked;
        uint32_t buffer_size    = BspUartUser_HandleTable[uart].tx_buffer_size / BSP_UART_TX_BUFFERS;
        uint32_t size_remaining = buffer_size - BspUartUser_HandleTable[uart].tx_buffer_write_count[unlocked];
        uint32_t write_count    = (uint32_t)size;
        uint32_t offset         = (uint32_t)((uint32_t)unlocked * buffer_size);
        uint8_t *buffer         = (uint8_t*)((uint32_t)BspUartUser_HandleTable[uart].tx_buffer + offset + BspUartUser_HandleTable[uart].tx_buffer_write_count[unlocked]);

        /* TODO SD-235 handle writing the reamining bytes */
        if (size_remaining < write_count)
        {
            write_count = size_remaining;
        }

        memcpy(buffer, data, write_count);
        BspUartUser_HandleTable[uart].tx_buffer_write_count[unlocked] += write_count;

        /* Critical section end */
        BspUartUser_HandleTable[uart].tx_lock = false;

        if (!BspUartUser_HandleTable[uart].txing)
        {
            uint8_t locked = BspUartUser_HandleTable[uart].tx_unlocked;
            BspUartUser_HandleTable[uart].tx_unlocked ^= 1U;

            BspUartUser_HandleTable[uart].txing = true;

            error = (Bsp_Error_t)HAL_UART_Transmit_DMA(BspUartUser_HandleTable[uart].uart_handle,
                                                       (uint8_t*)((uint32_t)BspUartUser_HandleTable[uart].tx_buffer + offset),
                                                       BspUartUser_HandleTable[uart].tx_buffer_write_count[locked]);
        }
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
        error = (Bsp_Error_t)HAL_UART_Receive_DMA(BspUartUser_HandleTable[uart].uart_handle, data, size);
    }

    return error;
}

static void BspUart_TxCallback(Bsp_UartHandle_t *uart_handle)
{
    Bsp_Uart_t* uart = BspUartUser_GetUart(uart_handle);

    if (NULL != uart)
    {
        /* Reset write count of buffer currently locked */
        uart->tx_buffer_write_count[uart->tx_unlocked ^ 1U] = 0U;

        if ((uart->tx_lock) || (0U == uart->tx_buffer_write_count[uart->tx_unlocked]))
        {
            uart->txing = false;
        }
        else
        {
            /* Toggle locked buffer */
            uint8_t tx_locked = uart->tx_unlocked;
            uart->tx_unlocked ^= 1U;

            uint32_t buffer_size = uart->tx_buffer_size / BSP_UART_TX_BUFFERS;

            (void)HAL_UART_Transmit_DMA(uart->uart_handle,
                                        (uint8_t*)((uint32_t)uart->tx_buffer + (uint32_t)((uint32_t)tx_locked * buffer_size)),
                                        uart->tx_buffer_write_count[tx_locked]);
        }

        if (NULL != uart->tx_callback)
        {
            uart->tx_callback(uart);
        }

    }
}

static void BspUart_RxCallback(Bsp_UartHandle_t *uart_handle)
{
    Bsp_Uart_t* uart = BspUartUser_GetUart(uart_handle);

    if ((NULL != uart) && (NULL != uart->rx_callback))
    {
        uart->rx_callback(uart);
    }
}