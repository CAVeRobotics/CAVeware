#include "bsp_uart_user.h"

#include <stddef.h>
#include <stdint.h>

#include "usart.h"

#include "bsp.h"

#define BSP_UART_USER_0_BUFFER_SIZE 1024U
#define BSP_UART_USER_1_BUFFER_SIZE 1024U

static uint8_t BspUartUser_LogTxBuffer[BSP_UART_USER_0_BUFFER_SIZE];
static uint8_t BspUartUser_CommsTxBuffer[BSP_UART_USER_1_BUFFER_SIZE];
static uint8_t BspUartUser_CommsRxBuffer[BSP_UART_USER_1_BUFFER_SIZE];

Bsp_Uart_t BspUartUser_HandleTable[BSP_UART_USER_MAX] = {
    [BSP_UART_USER_0] = {
        .uart_handle      = &huart6,
        .mode             = BSP_UART_MODE_TX,
        .tx_buffer        = BspUartUser_LogTxBuffer,
        .tx_buffer_size   = (uint32_t)sizeof(BspUartUser_LogTxBuffer),
        .tx_read_pointer  = 0U,
        .tx_write_pointer = 0U,
        .tx_reading       = 0U,
        .txing            = false,
        .rx_buffer        = NULL,
        .rx_buffer_size   = 0U,
        .read_pointer     = 0U,
    },
    [BSP_UART_USER_1] = {
        .uart_handle      = &huart1,
        .mode             = BSP_UART_MODE_RXTX,
        .tx_buffer        = BspUartUser_CommsTxBuffer,
        .tx_buffer_size   = (uint32_t)sizeof(BspUartUser_CommsTxBuffer),
        .tx_read_pointer  = 0U,
        .tx_write_pointer = 0U,
        .tx_reading       = 0U,
        .txing            = false,
        .rx_buffer        = BspUartUser_CommsRxBuffer,
        .rx_buffer_size   = (uint32_t)sizeof(BspUartUser_CommsRxBuffer),
        .read_pointer     = 0U,
    },
};