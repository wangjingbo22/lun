#ifndef WRAP_CAN_H
#define WRAP_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
} CANInstance;


/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
} CAN_Init_Config_s;

/**
 * @brief 初始化CAN外设
 * 
 */
void CANInit(void);

/**
 * @brief 配置can过滤器
 * 
 */
void CANFilterConfig(void);



#ifdef __cplusplus
}
#endif
#endif // WRAP_CAN_H