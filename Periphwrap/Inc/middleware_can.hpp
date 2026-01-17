#ifndef MIDDLEWARE_CAN_H
#define MIDDLEWARE_CAN_H
#include "can.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_can.h"
#include "wrap_can.h"




class MidCan
{
public:

/************************************获取can实例*********************************** */
/**
*@brief 单例模式，全局只有一个can实例，禁止拷贝构造和=，构造函数私有化
*
*/
inline static MidCan& getcan()
{
    return midcan;
}
MidCan& operator=(const MidCan&) = delete;
MidCan(const MidCan&) = delete;
/**************************************************************************** */


/**************************************can成员************************************ */

uint8_t rxbuff[8];
uint8_t txbuff[8];
CAN_RxHeaderTypeDef rxheader;
CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
CAN_TxMailBox_TypeDef txmailbox;
void decode(void);


private:
MidCan() 
{
    txconf.DLC = 8;
    txconf.IDE = CAN_ID_STD;
    txconf.RTR = CAN_RTR_DATA;
}
static MidCan midcan;
};




#endif // MIDDLEWARE_CAN_H