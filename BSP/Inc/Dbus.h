#ifndef DBUS_H
#define DBUS_H

#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
uint8_t s[2];
int16_t ch[4];
} Dbus_t;

extern Dbus_t DbusData;

/**
 * @brief 遥控器接收初始化
 * 
 */
void DbusInit(void);

/**
 * @brief 解析遥控器数据
 * 
 */
void DbusDecode(uint8_t* data);

/**
 * @brief 解析遥控器数据要用到的辅助函数，对通道数据进行偏移
 * 
 * @param rc 
 * @return int16_t 
 */
int16_t offset(int16_t rc);

/**
 * @brief 串口的空闲接收中断回调函数
 * 
 */
void DbusCallback(void);

#ifdef __cplusplus
}
#endif
#endif // DBUS_H