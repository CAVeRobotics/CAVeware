#ifndef BSP_LOGGER_H
#define BSP_LOGGER_H

#include <stdlib.h>

typedef enum
{
    BSP_LOGGER_LEVEL_ERROR,
    BSP_LOGGER_LEVEL_WARNING,
    BSP_LOGGER_LEVEL_INFO,
    BSP_LOGGER_LEVEL_DEBUG,
    BSP_LOGGER_LEVEL_VERBOSE
} BspLogger_Level_t;

void BspLogger_SetLogLevel(const BspLogger_Level_t level);
void BspLogger_RegisterCustomLogger(void (*log)(const char *const buffer, const size_t size), char *const buffer, const size_t size);
void BspLogger_Log(const char *const tag, const BspLogger_Level_t level, char *const format, ...);

#ifdef BSP_LOGGER
#define BSP_LOGGER_LOG_ERROR(tag, format, ...)   BspLogger_Log(tag, BSP_LOGGER_LEVEL_ERROR, format, ## __VA_ARGS__)
#define BSP_LOGGER_LOG_WARNING(tag, format, ...) BspLogger_Log(tag, BSP_LOGGER_LEVEL_WARNING, format, ## __VA_ARGS__)
#define BSP_LOGGER_LOG_INFO(tag, format, ...)    BspLogger_Log(tag, BSP_LOGGER_LEVEL_INFO, format, ## __VA_ARGS__)
#define BSP_LOGGER_LOG_DEBUG(tag, format, ...)   BspLogger_Log(tag, BSP_LOGGER_LEVEL_DEBUG, format, ## __VA_ARGS__)
#define BSP_LOGGER_LOG_VERBOSE(tag, format, ...) BspLogger_Log(tag, BSP_LOGGER_LEVEL_VERBOSE, format, ## __VA_ARGS__)
#else
#define BSP_LOGGER_LOG_ERROR(tag, format, ...)   ;
#define BSP_LOGGER_LOG_WARNING(tag, format, ...) ;
#define BSP_LOGGER_LOG_INFO(tag, format, ...)    ;
#define BSP_LOGGER_LOG_DEBUG(tag, format, ...)   ;
#define BSP_LOGGER_LOG_VERBOSE(tag, format, ...) ;
#endif

#endif /* BSP_LOGGER_H */