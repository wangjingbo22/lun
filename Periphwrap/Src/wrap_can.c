#include "wrap_can.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"



void CANFilterConfig(void)
{
    // 配置CAN过滤器的代码
    CAN_FilterTypeDef can_filter_conf;
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;           // 掩码模式
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;          // 32位过滤器
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO0;          // 分配到 FIFO0
    can_filter_conf.SlaveStartFilterBank = 14;                    // CAN2 从第14个过滤器开始
    can_filter_conf.FilterBank = 0;                               // 使用过滤器0
    can_filter_conf.FilterIdHigh = 0x0000;                        // 过滤器ID高16位
    can_filter_conf.FilterIdLow = 0x0000;                         // 过滤器ID低16位
    can_filter_conf.FilterMaskIdHigh = 0x0000;                    // 掩码高16位 (全0表示接收所有ID)
    can_filter_conf.FilterMaskIdLow = 0x0000;                     // 掩码低16位 (全0表示接收所有ID)
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;         // 启用过滤器
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_conf);
}

void CANInit(void)
{
    // 1. 先配置过滤器
    CANFilterConfig();
    
    // 2. 启动 CAN
    HAL_CAN_Start(&hcan1);
    
    // 3. 激活接收中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


