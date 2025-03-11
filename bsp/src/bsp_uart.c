#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_uart_user.h"

extern void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);

static void BspUart_TxCallback(Bsp_UartHandle_t *uart_handle);
static Bsp_Error_t BspUart_TxBufferWrite(Bsp_Uart_t *const uart, const uint8_t *const data, const uint32_t size);
static Bsp_Error_t BspUart_TxBufferStartRead(Bsp_Uart_t *const uart);
static Bsp_Error_t BspUart_TxBufferReadComplete(Bsp_Uart_t *const uart);
static Bsp_Error_t BspUart_ResetBuffer(Bsp_UartDoubleBuffer_t *const double_buffer);
static inline uint8_t BspUart_ToggleLockedBuffer(Bsp_UartDoubleBuffer_t *const double_buffer);

Bsp_Error_t BspUart_Start(const BspUartUser_Uart_t uart)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (uart < BSP_UART_USER_MAX)
    {
        (void)BspUart_ResetBuffer(&BspUartUser_HandleTable[uart].tx_buffer);

        BspUartUser_HandleTable[uart].read_pointer = 0U;

        /* TODO SD-234 receive, error, and abort callbacks */
        BspUartUser_HandleTable[uart].uart_handle->TxCpltCallback = BspUart_TxCallback;

        HAL_UART_MspInit(BspUartUser_HandleTable[uart].uart_handle);

        /* TODO SD-241 HAL_UART_Receive_DMA size is UINT16_MAX */
        HAL_UART_Receive_DMA(BspUartUser_HandleTable[uart].uart_handle, BspUartUser_HandleTable[uart].rx_buffer, BspUartUser_HandleTable[uart].rx_buffer_size);
    }

    return error;
}

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
        error = BspUart_TxBufferWrite(&BspUartUser_HandleTable[uart], data, size);
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
    else if (uart >= BSP_UART_USER_MAX)
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
        (void)BspUart_TxBufferReadComplete(uart);
    }
}

static Bsp_Error_t BspUart_TxBufferWrite(Bsp_Uart_t *const uart, const uint8_t *const data, const uint32_t size)
{
    Bsp_Error_t error = BSP_ERROR_NULL;

    if ((NULL == uart) || (NULL == uart->tx_buffer.buffer) || (NULL == data))
    {
    }
    else
    {
        Bsp_UartDoubleBuffer_t *double_buffer = &uart->tx_buffer;

        /* Critical section start */
        double_buffer->writing = true;

        uint8_t  unlocked       = double_buffer->unlocked;
        uint32_t offset         = (uint32_t)((uint32_t)unlocked * double_buffer->half_buffer_size);
        uint32_t size_remaining = double_buffer->half_buffer_size - double_buffer->write_count[unlocked];
        uint8_t *buffer_write   = (uint8_t *)((uint32_t)double_buffer->buffer + offset + double_buffer->write_count[unlocked]);
        uint32_t bytes_to_write = size;

        /* TODO SD-235 handle truncated bytes */
        if (size_remaining < bytes_to_write)
        {
            bytes_to_write = size_remaining;
        }

        (void)memcpy(buffer_write, data, bytes_to_write);
        double_buffer->write_count[unlocked] += bytes_to_write;

        /* Critical section end */
        double_buffer->writing = false;

        error = BspUart_TxBufferStartRead(uart);
    }

    return error;
}

static Bsp_Error_t BspUart_TxBufferStartRead(Bsp_Uart_t *const uart)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if ((NULL == uart) || (NULL == uart->uart_handle) || (NULL == uart->tx_buffer.buffer))
    {
        error = BSP_ERROR_NULL;
    }
    else if ((!uart->tx_buffer.reading) && (uart->tx_buffer.write_count[uart->tx_buffer.unlocked] > 0U))
    {
        Bsp_UartDoubleBuffer_t *double_buffer = &uart->tx_buffer;

        uint8_t locked = BspUart_ToggleLockedBuffer(double_buffer);

        double_buffer->reading            = true;
        double_buffer->read_count[locked] = 0U;

        error = (Bsp_Error_t)HAL_UART_Transmit_DMA(uart->uart_handle,
                                                   (uint8_t *)((uint32_t)double_buffer->buffer + (uint32_t)((uint32_t)locked * double_buffer->half_buffer_size)),
                                                   double_buffer->write_count[locked]);
    }

    return error;
}

static Bsp_Error_t BspUart_TxBufferReadComplete(Bsp_Uart_t *const uart)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (NULL == uart)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        Bsp_UartDoubleBuffer_t *double_buffer = &uart->tx_buffer;

        uint8_t locked = double_buffer->unlocked ^ 1U;

        double_buffer->read_count[locked]  = double_buffer->write_count[locked];
        double_buffer->write_count[locked] = 0U;
        double_buffer->reading             = false;

        if (!double_buffer->writing)
        {
            error = BspUart_TxBufferStartRead(uart);
        }
    }

    return error;
}

static Bsp_Error_t BspUart_ResetBuffer(Bsp_UartDoubleBuffer_t *const double_buffer)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (NULL == double_buffer)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        double_buffer->writing  = false;
        double_buffer->reading  = false;
        double_buffer->unlocked = 0U;

        for (size_t i = 0U; i < (sizeof(double_buffer->write_count) / sizeof(double_buffer->write_count[0])); i++)
        {
            double_buffer->write_count[i] = 0U;
            double_buffer->read_count[i]  = 0U;
        }
    }

    return error;
}

static inline uint8_t BspUart_ToggleLockedBuffer(Bsp_UartDoubleBuffer_t *const double_buffer)
{
    uint8_t locked = double_buffer->unlocked;
    double_buffer->unlocked ^= 1U;

    return locked;
}