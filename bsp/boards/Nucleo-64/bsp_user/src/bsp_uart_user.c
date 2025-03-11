#include "bsp_uart_user.h"

#include <stddef.h>
#include <stdint.h>

#include "usart.h"

#include "bsp.h"

#define BSP_UART_USER_LOG_BUFFER_SIZE   1024U
#define BSP_UART_USER_COMMS_BUFFER_SIZE 1024U

static uint8_t BspUartUser_LogTxBuffer[BSP_UART_USER_LOG_BUFFER_SIZE];
static uint8_t BspUartUser_LogRxBuffer[BSP_UART_USER_LOG_BUFFER_SIZE];
static uint8_t BspUartUser_CommsTxBuffer[BSP_UART_USER_COMMS_BUFFER_SIZE];
static uint8_t BspUartUser_CommsRxBuffer[BSP_UART_USER_COMMS_BUFFER_SIZE];

Bsp_Uart_t BspUartUser_HandleTable[BSP_UART_USER_MAX] = {
    [BSP_UART_USER_LOG] = {
        .uart_handle = &huart2,
        .tx_buffer   = {
            .buffer           = BspUartUser_LogTxBuffer,
            .half_buffer_size = sizeof(BspUartUser_LogTxBuffer) >> 1U,
            .writing          = false,
            .reading          = false,
            .unlocked         = 0U,
            .write_count      = {
                0U, 0U
            },
            .read_count = {
                0U, 0U
            },
        },
        .rx_buffer      = BspUartUser_LogRxBuffer,
        .rx_buffer_size = (uint32_t)sizeof(BspUartUser_LogRxBuffer),
        .read_pointer   = 0U,
    },
    [BSP_UART_USER_COMMS] = {
        .uart_handle = &huart6,
        .tx_buffer   = {
            .buffer           = BspUartUser_CommsTxBuffer,
            .half_buffer_size = sizeof(BspUartUser_CommsTxBuffer) >> 1U,
            .writing          = false,
            .reading          = false,
            .unlocked         = 0U,
            .write_count      = {
                0U, 0U
            },
            .read_count = {
                0U, 0U
            },
        },
        .rx_buffer      = BspUartUser_CommsRxBuffer,
        .rx_buffer_size = (uint32_t)sizeof(BspUartUser_CommsRxBuffer),
        .read_pointer   = 0U,
    }
};

Bsp_Uart_t* BspUartUser_GetUart(const Bsp_UartHandle_t *const uart_handle)
{
    Bsp_Uart_t* uart = NULL;

    if (uart_handle == BspUartUser_HandleTable[BSP_UART_USER_LOG].uart_handle)
    {
        uart = &BspUartUser_HandleTable[BSP_UART_USER_LOG];
    }
    else if (uart_handle == BspUartUser_HandleTable[BSP_UART_USER_COMMS].uart_handle)
    {
        uart = &BspUartUser_HandleTable[BSP_UART_USER_COMMS];
    }

    return uart;
}