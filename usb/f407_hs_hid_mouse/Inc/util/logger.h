#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdint.h>

typedef enum{
    LOG_LEVEL_ERROR,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} LOG_LEVEL_e;

/// The global variable `System_Log_Level` should be defined and given the desired log level.
extern LOG_LEVEL_e System_Log_Level;

void log_error(const char * const format, ...);
void log_info(const char *  const format, ...);
void log_debug(const char * const format, ...);
void log_debug_array(const char * const label, const void *array, const uint16_t len);

#endif
