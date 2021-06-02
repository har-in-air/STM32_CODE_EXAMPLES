#include "../../inc/util/logger.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include "../../inc/cmsis/device/stm32f4xx.h"

LOG_LEVEL_e System_Log_Level = LOG_LEVEL_INFO;

static void _log(const LOG_LEVEL_e log_level, const char * const format, va_list args);
static const char* const _get_log_level_string(const LOG_LEVEL_e log_level);

// Redirects `printf()` output to the serial wire out (SWO).
// This function overrides a weak function symbol and is not to be used directly.
int _write(int file, char *ptr, int len){
	while (len--){
		ITM_SendChar(*ptr++);
		}
  	return len;
	}


static const char* const _get_log_level_string(const LOG_LEVEL_e log_level){
    switch(log_level)    {
        case LOG_LEVEL_ERROR:
            return "ERROR";
        case LOG_LEVEL_INFO:
            return "INFO";
        case LOG_LEVEL_DEBUG:
            return "DEBUG";
        default :
        	return "UNKNOWN";
    	}
	}


static void _log(const LOG_LEVEL_e log_level, const char  * const format, va_list args){
    if (log_level > System_Log_Level){
        return;
    	}
	printf("[%s] ", _get_log_level_string(log_level));
	vfprintf(stdout, format, args);
	printf("\n");
	}


void log_error(const char * const format, ...){
    va_list args;
	va_start(args, format);
    _log(LOG_LEVEL_ERROR, format, args);
    va_end(args);
	}


void log_info(const char * const format, ...){
    va_list args;
	va_start(args, format);
    _log(LOG_LEVEL_INFO, format, args);
    va_end(args);
	}


void log_debug(const char * const format, ...){
    va_list args;
	va_start(args, format);
    _log(LOG_LEVEL_DEBUG, format, args);
    va_end(args);
	}


// Log the content of an array.
// label The label of the array.
// array Pointer to the array.
// len The length of data in bytes.
void log_debug_array(const char * const label, const void *array, const uint16_t len){
    if (LOG_LEVEL_DEBUG > System_Log_Level){
        return;
    	}
	printf("[%s] %s[%d]: {", _get_log_level_string(LOG_LEVEL_DEBUG), label, len);
    for (uint16_t inx = 0; inx < len; inx++){
    	uint8_t val = *((uint8_t *)(array + inx));
    	printf("0x%02X", val);
    	// Add ", " after all elements except the last one.
    	if (inx < len - 1) 	{
    	    printf(", ");
    		}
    	}
	printf("}\n");
	}
