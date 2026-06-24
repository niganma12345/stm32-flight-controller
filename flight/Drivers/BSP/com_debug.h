#ifndef COM_DEBUG_H
#define COM_DEBUG_H

#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
//日志输出打印开关
#define DEBUG_LOGENABLE 1
#if DEBUG_LOGENABLE
#define debug_printf(format, ...) printf("[%s:%d] "format, __FUNCTION__, __LINE__ ,##__VA_ARGS__)
#else
#define debug_printf(format, ...)

#endif
#endif /* COM_DEBUG_H */
