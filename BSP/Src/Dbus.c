#include "Dbus.h"
#include "usart.h"
#include "wrap_uart.h"
#include <stdint.h>
#include <string.h>

uint8_t rxBuff_rc[54];
uint8_t* pdata = rxBuff_rc;
Dbus_t DbusData;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


void DbusInit(void)
{
    //memset(&rc,0,sizeof(rc));
   
    for(int i = 0;i<4;++i)
    {
        DbusData.ch[i] = 0;
    }
    DbusData.s[0] = 0;
    DbusData.s[1] = 0;
    //开启串口空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    //启动串口
    if(huart3.RxState == HAL_UART_STATE_READY)
    {
        huart3.pRxBuffPtr = rxBuff_rc;
        huart3.RxXferSize = 54;
        huart3.ErrorCode = HAL_UART_ERROR_NONE;
        //配置DMA
        HAL_DMA_Start(huart3.hdmarx,(uint32_t)&huart3.Instance -> DR,(uint32_t)rxBuff_rc,54);
        //使能串口DMA接受
        SET_BIT(huart3.Instance -> CR3,USART_CR3_DMAR);
    }
}


void DbusDecode(uint8_t* data)
{
    DbusData.ch[0] = offset(((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF);
    DbusData.ch[1] = offset((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF);
    DbusData.ch[2] = offset((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF);
    DbusData.ch[3] = offset((((int16_t)data[4] >> 1) | ((int16_t)data[5]<<7)) & 0x07FF); 

    DbusData.s[0] = (data[5] >> 6) & 0x03;
    DbusData.s[1] =  (data[5] >> 4) & 0x03;
}

// void rc_processdata(uint8_t* rxBuff)
// {
//     rc.ch[0] = offset(((int16_t)rxBuff[0] | ((int16_t)rxBuff[1] << 8)) & 0x07FF);
//     rc.ch[1] = offset((((int16_t)rxBuff[1] >> 3) | ((int16_t)rxBuff[2] << 5)) & 0x07FF);
//     rc.ch[2] = offset((((int16_t)rxBuff[2] >> 6) | ((int16_t)rxBuff[3] << 2) | ((int16_t)rxBuff[4] << 10)) & 0x07FF);
//     rc.ch[3] = offset((((int16_t)rxBuff[4] >> 1) | ((int16_t)rxBuff[5]<<7)) & 0x07FF); 

//     rc.s[0] = (rxBuff[5] >> 6) & 0x03;
//     rc.s[1] =  (rxBuff[5] >> 4) & 0x03;
// }

int16_t offset(int16_t rc)
{
    int16_t temp;
   temp = rc-1024;
    return temp;
}

void DbusCallback(void)
{
    // 1. 先判断是否是空闲中断
    if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        // 2. 清除空闲中断标志 (F4系列必须通过读SR+读DR来清除，或者使用专用宏)
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        
        // 3. 暂停 DMA 以便处理数据
        __HAL_DMA_DISABLE(huart3.hdmarx);
        
        // 4. 校验接收长度 (可选，但推荐)
        // 54 - 剩余计数 = 实际接收长度
        // DBUS 协议一帧通常是 18 字节
        if ((54 - __HAL_DMA_GET_COUNTER(huart3.hdmarx)) == 18)
        {
            // 5. 解码数据
            // 直接使用全局数组 rxBuff_rc，不要使用 huart3.pRxBuffPtr (它可能未被正确初始化或更新)
            DbusDecode(rxBuff_rc);
        }
        
        // 6. 重置 DMA 计数器并重新启动
        __HAL_DMA_SET_COUNTER(huart3.hdmarx, 54);
        __HAL_DMA_ENABLE(huart3.hdmarx);
    }
}

