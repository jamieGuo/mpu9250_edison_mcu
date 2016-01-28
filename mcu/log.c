/*
 * log.c
 *
 *  Created on: 2015/12/7
 *      Author: KuoJamie
 */


#include "log.h"

#include <stdarg.h>
#include "mcu_api.h"
#include "mcu_errno.h"

//#define _MLPrintLog(a, b, fmt, __VA_ARGS__)	debug_print(DBG_INFO, __VA_ARGS__);

int _MLPrintLog(int priority, const char *tag, const char *fmt,	...)	{

	va_list args;
	va_start (args, fmt);
	debug_print(DBG_INFO, fmt, args);
	va_end (args);
	return 0;
}
int _MLPrintVaLog(int priority, const char *tag, const char *fmt, va_list args)	{

	debug_print(DBG_INFO, fmt, args);
	return 0;
}
