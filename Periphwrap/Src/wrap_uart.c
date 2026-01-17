#include "wrap_uart.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "dma.h"

// 定义日志缓冲区大小
#define LOG_BUFFER_SIZE 128

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



void log_printf(const char *fmt, ...)
{
    char buffer[LOG_BUFFER_SIZE];
    va_list args;

    // 开始处理变参
    va_start(args, fmt);
    
    // 格式化字符串
    // vsnprintf 会防止缓冲区溢出
    // 已在 CMake 中开启 -u _printf_float，支持浮点数打印
    //在gcc-arm-none-eabi.cmake中添加下面这一行
    //set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
    vsnprintf(buffer, LOG_BUFFER_SIZE, fmt, args);
    
    // 结束处理变参
    va_end(args);
    
    // 通过串口发送
    // 使用阻塞模式发送，超时时间设为 10ms
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), 10);
}

