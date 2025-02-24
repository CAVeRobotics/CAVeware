#include "bsp_logger.h"

#ifdef BSP_LOGGER
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#endif /* BSP_LOGGER */

#include "bsp.h"
#include "bsp_tick.h"

#define BSP_LOGGER_ANSI_RESET  "\x1B[0m"
#define BSP_LOGGER_ANSI_RED    "\x1B[0;31m"
#define BSP_LOGGER_ANSI_YELLOW "\x1B[0;33m"
#define BSP_LOGGER_ANSI_GREEN  "\x1B[0;32m"
#define BSP_LOGGER_ANSI_BLUE   "\x1B[0;34m"
#define BSP_LOGGER_ANSI_PURPLE "\x1B[0;35m"

typedef enum
{
    BSP_LOGGER_MODE_PRINT,
    BSP_LOGGER_MODE_CUSTOM
} BspLogger_Mode_t;

#ifdef BSP_LOGGER
static const char *kBspLogger_AnsiColorLut[] = {
    BSP_LOGGER_ANSI_RED, BSP_LOGGER_ANSI_YELLOW, BSP_LOGGER_ANSI_GREEN, BSP_LOGGER_ANSI_BLUE, BSP_LOGGER_ANSI_PURPLE
};

static const char *kBspLogger_PrefixFormat  = "%s[%010u] [%s] ";
static const char *kBspLogger_PostfixFormat = "%s\r\n";

static BspLogger_Mode_t  BspLogger_Mode  = BSP_LOGGER_MODE_PRINT;
static BspLogger_Level_t BspLogger_Level = BSP_LOGGER_LEVEL_VERBOSE;

static void   (*BspLogger_CustomLog)(const char *const buffer, const size_t size) = NULL; /* Client-specified logging function, not be called directly */
static char * BspLogger_CustomLogBuffer     = NULL;
static size_t BspLogger_CustomLogBufferSize = 0UL;
#endif /* BSP_LOGGER */

#ifdef BSP_LOGGER
static inline void BspLogger_Print(const char *const tag, const BspLogger_Level_t level, const char *const format, va_list args);
static inline void BspLogger_Custom(const char *const tag, const BspLogger_Level_t level, const char *const format, va_list args);
#endif /* BSP_LOGGER */

void BspLogger_SetLogLevel(const BspLogger_Level_t level)
{
#ifdef BSP_LOGGER
    BspLogger_Level = level;
#else
    BSP_UNUSED(level);
#endif /* BSP_LOGGER */
}

void BspLogger_RegisterCustomLogger(void (*log)(const char *const buffer, const size_t size), char *const buffer, const size_t size)
{
#ifdef BSP_LOGGER
    if ((NULL != log) && (NULL != buffer))
    {
        BspLogger_CustomLog           = log;
        BspLogger_CustomLogBuffer     = buffer;
        BspLogger_CustomLogBufferSize = size;
        BspLogger_Mode                = BSP_LOGGER_MODE_CUSTOM;
    }
#else
    BSP_UNUSED(log);
    BSP_UNUSED(buffer);
    BSP_UNUSED(size);
#endif /* BSP_LOGGER */
}

void BspLogger_Log(const char *const tag, const BspLogger_Level_t level, char *const format, ...)
{
#ifdef BSP_LOGGER
    if ((NULL != tag) && (NULL != format) && (level <= BspLogger_Level))
    {
        va_list args;

        va_start(args, format);

        switch (BspLogger_Mode)
        {
        case BSP_LOGGER_MODE_PRINT:
            BspLogger_Print(tag, level, format, args);
            break;
        case BSP_LOGGER_MODE_CUSTOM:
            BspLogger_Custom(tag, level, format, args);
            break;
        default:
            break;
        }

        va_end(args);
    }
#else
    BSP_UNUSED(tag);
    BSP_UNUSED(level);
    BSP_UNUSED(format);
#endif /* BSP_LOGGER */
}

#ifdef BSP_LOGGER
static inline void BspLogger_Print(const char *const tag, const BspLogger_Level_t level, const char *const format, va_list args)
{
    printf(kBspLogger_PrefixFormat, kBspLogger_AnsiColorLut[level], BspTick_GetTick(), tag);
    vprintf(format, args);
    printf(kBspLogger_PostfixFormat, BSP_LOGGER_ANSI_RESET);
}

static inline void BspLogger_Custom(const char *const tag, const BspLogger_Level_t level, const char *const format, va_list args)
{
    size_t  bytes_written = 0U;
    int64_t size          = snprintf(BspLogger_CustomLogBuffer, BspLogger_CustomLogBufferSize, kBspLogger_PrefixFormat, kBspLogger_AnsiColorLut[level], BspTick_GetTick(), tag);

    if ((size > 0L) && (size < BspLogger_CustomLogBufferSize))
    {
        bytes_written += (size_t)size;

        size = vsnprintf((BspLogger_CustomLogBuffer + bytes_written), (BspLogger_CustomLogBufferSize - bytes_written), format, args);
    }

    if ((size > 0L) && (size < (BspLogger_CustomLogBufferSize - bytes_written)))
    {
        bytes_written += (size_t)size;

        size = snprintf((BspLogger_CustomLogBuffer + bytes_written), (BspLogger_CustomLogBufferSize - bytes_written), kBspLogger_PostfixFormat, BSP_LOGGER_ANSI_RESET);
    }

    if ((size > 0L) && (size < (BspLogger_CustomLogBufferSize - bytes_written)))
    {
        bytes_written += (size_t)size;
    }

    BspLogger_CustomLog(BspLogger_CustomLogBuffer, bytes_written);
}
#endif /* BSP_LOGGER */