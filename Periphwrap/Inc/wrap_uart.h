#ifndef WRAP_UART_H
#define WRAP_UART_H


#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
#include "usart.h"


/**
 * @brief 将传入的数据打印到终端
 * 
 * @param fmt 
 * @param ... 
 * @note 打印简单的字符串
 *   log_printf("System Init OK\r\n");
 * 
 *   打印变量
 *   int count = 10;
 *   float voltage = 3.3f;
 *   log_printf("Count: %d, Voltage: %.2f V\r\n", count, voltage);
 *   在你的 test 任务中可以这样用：
 *   log_printf("Roll:%d.%02d Pitch:%d.%02d\r\n", (int)roll, (int)((roll-(int)roll)*100));
 */
void log_printf(const char *fmt, ...);


#ifdef __cplusplus
}
#endif
#endif // WRAP_UART_H