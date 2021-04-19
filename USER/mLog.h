#ifndef MLOG_H_
#define MLOG_H_

#include "usart.h"

#ifndef DEBUG_INFO
#define DEBUG_INFO
#endif

#ifdef DEBUG_INFO
#define user_main_printf(format, ...) 	USARTx_printf(USART1, format "\r\n", ##__VA_ARGS__)
#define user_main_info(format, ...) 		USARTx_printf(USART1, "[INFO] [%s@%s,%d] " format "\r\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__);
#define user_main_debug(format, ...)    USARTx_printf(USART1, "[DEBUG] [%s@%s,%d] " format "\r\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#define user_main_error(format, ...) 		USARTx_printf(USART1, "[ERROR] [%s@%s,%d] " format "\r\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define user_main_printf(format, ...)
#define user_main_info(format, ...)
#define user_main_debug(format, ...)
#define user_main_error(format, ...)
#endif

#endif
