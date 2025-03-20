#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_logger.h"
#include "bsp_uart_user.h"

static const char *kBspUart_LogTag = "BSP UART";

extern void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);

static void BspUart_TxCallback(Bsp_UartHandle_t *uart_handle);
static Bsp_Error_t BspUart_StartTransmit(Bsp_Uart_t *const uart);
static void BspUart_ErrorCallback(Bsp_UartHandle_t *uart_handle);

Bsp_Error_t BspUart_Start(const BspUartUser_Uart_t uart)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (uart < BSP_UART_USER_MAX)
    {
        BspUartUser_HandleTable[uart].tx_read_pointer  = 0U;
        BspUartUser_HandleTable[uart].tx_write_pointer = 0U;
        BspUartUser_HandleTable[uart].tx_reading       = 0U;
        BspUartUser_HandleTable[uart].txing            = false;

        BspUartUser_HandleTable[uart].read_pointer = 0U;

        /* TODO SD-234 receive, error, and abort callbacks */
        BspUartUser_HandleTable[uart].uart_handle->TxCpltCallback = BspUart_TxCallback;
        BspUartUser_HandleTable[uart].uart_handle->ErrorCallback  = BspUart_ErrorCallback;

        HAL_UART_MspInit(BspUartUser_HandleTable[uart].uart_handle);

        if (BSP_UART_MODE_TX != BspUartUser_HandleTable[uart].mode)
        {
            error = (Bsp_Error_t)HAL_UART_Receive_DMA(BspUartUser_HandleTable[uart].uart_handle,
                                                      BspUartUser_HandleTable[uart].rx_buffer,
                                                      (uint16_t)BspUartUser_HandleTable[uart].rx_buffer_size);
        }
        else
        {
            error = BSP_ERROR_NONE;
        }
    }

    return error;
}

Bsp_Error_t BspUart_Stop(const BspUartUser_Uart_t uart)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (uart < BSP_UART_USER_MAX)
    {
        error = (Bsp_Error_t)HAL_UART_DMAStop(BspUartUser_HandleTable[uart].uart_handle);
    }

    return error;
}

Bsp_Error_t BspUart_Transmit(const BspUartUser_Uart_t uart, const uint8_t *const data, const size_t size)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if (NULL == data)
    {
    }
    else if ((uart >= BSP_UART_USER_MAX) || (BSP_UART_MODE_RX == BspUartUser_HandleTable[uart].mode))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        uint32_t read_pointer    = BspUartUser_HandleTable[uart].tx_read_pointer;
        uint32_t write_pointer   = BspUartUser_HandleTable[uart].tx_write_pointer;
        uint32_t bytes_remaining = BspUartUser_HandleTable[uart].tx_buffer_size;

        if (write_pointer != read_pointer)
        {
            bytes_remaining = (BspUartUser_HandleTable[uart].tx_buffer_size - (write_pointer - read_pointer)) % BspUartUser_HandleTable[uart].tx_buffer_size;
        }

        if (size > bytes_remaining)
        {
            error = BSP_ERROR_SIZE;
        }
        else
        {
            if ((write_pointer + size) > BspUartUser_HandleTable[uart].tx_buffer_size)
            {
                uint32_t bytes_to_end = BspUartUser_HandleTable[uart].tx_buffer_size - write_pointer;
                memcpy((uint8_t *)((uint32_t)BspUartUser_HandleTable[uart].tx_buffer + write_pointer), data, bytes_to_end);
                memcpy(BspUartUser_HandleTable[uart].tx_buffer, (uint8_t*)((uint32_t)data + bytes_to_end), (uint16_t)(size - bytes_to_end));
            }
            else
            {
                memcpy((uint8_t *)((uint32_t)BspUartUser_HandleTable[uart].tx_buffer + write_pointer), data, size);
            }
            BspUartUser_HandleTable[uart].tx_write_pointer += (uint32_t)size;
            BspUartUser_HandleTable[uart].tx_write_pointer %= BspUartUser_HandleTable[uart].tx_buffer_size;

            error = BspUart_StartTransmit(&BspUartUser_HandleTable[uart]);
        }
    }

    return error;
}

Bsp_Error_t BspUart_Receive(const BspUartUser_Uart_t uart, uint8_t *const data, const size_t size, size_t *const bytes_read)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if ((NULL == data) || (NULL == bytes_read))
    {
        error = BSP_ERROR_NULL;
    }
    else if ((uart >= BSP_UART_USER_MAX) || (BSP_UART_MODE_TX == BspUartUser_HandleTable[uart].mode))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        uint32_t read_pointer  = BspUartUser_HandleTable[uart].read_pointer;
        uint32_t write_pointer = BspUartUser_HandleTable[uart].rx_buffer_size - BspUartUser_HandleTable[uart].uart_handle->hdmarx->Instance->NDTR;
        *bytes_read = (size_t)((write_pointer - read_pointer) % BspUartUser_HandleTable[uart].rx_buffer_size);

        if (*bytes_read > size)
        {
            *bytes_read = size;
        }

        if ((read_pointer + *bytes_read) > BspUartUser_HandleTable[uart].rx_buffer_size)
        {
            uint32_t bytes_to_end = BspUartUser_HandleTable[uart].rx_buffer_size - read_pointer;
            memcpy(data, (uint8_t*)((uint32_t)BspUartUser_HandleTable[uart].rx_buffer + read_pointer), bytes_to_end);
            memcpy((uint8_t*)((uint32_t)data + bytes_to_end), BspUartUser_HandleTable[uart].rx_buffer, write_pointer);
        }
        else
        {
            memcpy(data, (uint8_t*)((uint32_t)BspUartUser_HandleTable[uart].rx_buffer + read_pointer), *bytes_read);
        }

        BspUartUser_HandleTable[uart].read_pointer += *bytes_read;
        BspUartUser_HandleTable[uart].read_pointer %= BspUartUser_HandleTable[uart].rx_buffer_size;
    }

    return error;
}

static void BspUart_TxCallback(Bsp_UartHandle_t *uart_handle)
{
    Bsp_Uart_t* uart = BspUartUser_GetUart(uart_handle);

    if (NULL != uart)
    {
        uart->tx_read_pointer += uart->tx_reading;
        uart->tx_read_pointer %= uart->tx_buffer_size;
        uart->tx_reading       = 0U;
        uart->txing            = false;

        (void)BspUart_StartTransmit(uart);
    }
}

static Bsp_Error_t BspUart_StartTransmit(Bsp_Uart_t *const uart)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (NULL == uart)
    {
        error = BSP_ERROR_NULL;
    }
    else if (uart->txing)
    {
    }
    else
    {
        uint32_t read_pointer  = uart->tx_read_pointer;
        uint32_t write_pointer = uart->tx_write_pointer;
        uint32_t bytes_read    = 0U;

        if (read_pointer > write_pointer)
        {
            /* Transmit to end, next transmit will transmit remaining bytes at start of buffer */
            bytes_read = uart->tx_buffer_size - read_pointer;
        }
        else
        {
            bytes_read = write_pointer - read_pointer;
        }

        if (bytes_read > 0U)
        {
            error = (Bsp_Error_t)HAL_UART_Transmit_DMA(uart->uart_handle, (uint8_t*)((uint32_t)uart->tx_buffer + read_pointer), (uint16_t)bytes_read);

            if (BSP_ERROR_NONE == error)
            {
                uart->tx_reading = bytes_read;
                uart->txing      = true;
            }
        }
    }

    return error;
}

static void BspUart_ErrorCallback(Bsp_UartHandle_t *uart_handle)
{
    const Bsp_Uart_t* const uart = BspUartUser_GetUart(uart_handle);

    if (NULL != uart)
    {
        BSP_LOGGER_LOG_ERROR(kBspUart_LogTag, "Error detected");
    }
}