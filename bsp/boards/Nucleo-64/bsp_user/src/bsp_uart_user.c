#include "bsp_uart_user.h"

#include <stddef.h>
#include <stdint.h>

#include "usart.h"

#include "bsp.h"

#define BSP_UART_USER_LOG_TX_BUFFER_SIZE   1024U
#define BSP_UART_USER_COMMS_TX_BUFFER_SIZE 1024U

static uint8_t BspUartUser_LogTxBuffer[BSP_UART_USER_LOG_TX_BUFFER_SIZE];
static uint8_t BspUartUser_CommsTxBuffer[BSP_UART_USER_COMMS_TX_BUFFER_SIZE];

Bsp_Uart_t BspUartUser_HandleTable[BSP_UART_USER_MAX] = {
    [BSP_UART_USER_LOG] = {
        .uart_handle           = &huart2,
        .tx_buffer             = BspUartUser_LogTxBuffer,
        .tx_buffer_size        = (uint32_t)sizeof(BspUartUser_LogTxBuffer),
        .tx_lock               = false,
        .txing                 = false,
        .tx_unlocked           = 0U,
        .tx_buffer_write_count = {
            0U, 0U
        },
        .tx_callback = NULL,
        .rx_callback = NULL,
    },
    [BSP_UART_USER_COMMS] = {
        .uart_handle           = &huart6,
        .tx_buffer             = BspUartUser_CommsTxBuffer,
        .tx_buffer_size        = (uint32_t)sizeof(BspUartUser_CommsTxBuffer),
        .tx_lock               = false,
        .txing                 = false,
        .tx_unlocked           = 0U,
        .tx_buffer_write_count = {
            0U, 0U
        },
        .tx_callback = NULL,
        .rx_callback = NULL,
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