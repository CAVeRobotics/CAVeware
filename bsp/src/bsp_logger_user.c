#include "bsp_logger_user.h"

#ifdef BSP_LOGGER_USER_CUSTOM_LOGGER
#include <stddef.h>

#include "bsp_logger.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

#define BSP_LOGGER_USER_LOG_BUFFER_SIZE 1024U

static char BspLoggerUser_LogBuffer[BSP_LOGGER_USER_LOG_BUFFER_SIZE];

static void BspLoggerUser_CustomLogger(const char *const buffer, const size_t size);
#endif /* BSP_LOGGER_USER_CUSTOM_LOGGER */

void BspLoggerUser_RegisterCustomLogger(void)
{
#ifdef BSP_LOGGER_USER_CUSTOM_LOGGER
    BspLogger_RegisterCustomLogger(BspLoggerUser_CustomLogger, BspLoggerUser_LogBuffer, sizeof(BspLoggerUser_LogBuffer));
#endif /* BSP_LOGGER_USER_CUSTOM_LOGGER */
}

#ifdef BSP_LOGGER_USER_CUSTOM_LOGGER
static void BspLoggerUser_CustomLogger(const char *const buffer, const size_t size)
{
    (void)BspUart_Transmit(BSP_UART_USER_LOG, (uint8_t*)buffer, size);
}
#endif /* BSP_LOGGER_USER_CUSTOM_LOGGER */