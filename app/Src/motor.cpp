#include "motor.hpp"
#include "stm32f4xx_hal_can.h"
#include "vector"
#include "wrap_uart.h"
#include <algorithm>
#include <cstddef>
#include <vector>

uint8_t motor::tx[8] = {0};

/**
 * @brief 下标2是左驱动电机，下标5是右驱动电机
 *        设置腿长最短的编码器位置为0，1号变大，2号变小，绝对值相等
 */
std::vector<motor> motor_ = 
{
    {0x101, -4.092f, 9.69f, 0.04f, 1},
    {0x102, -5.189f, 9.69f, 0.04f, 1},
    {0x103, -0.101f, 9.69f, 0.04f, 1},
    {0x105, -0.616f, 9.69f, 0.04f, -1},
    {0x106, -0.632f, 9.69f, 0.04f, -1},
    {0x107, -0.107f, 9.69f, 0.04f, -1},
};

void motor::Motorsend(uint8_t* tx)
{

	*(int16_t *)&tx[0] = ((int16_t)(motor_[0].voltage * 1000));
	*(int16_t *)&tx[2] = ((int16_t)(motor_[1].voltage * 1000));
	*(int16_t *)&tx[4] = ((int16_t)(motor_[2].voltage * 1000));
	sendframe(0x100, tx);

	*(int16_t *)&tx[0] = ((int16_t)(motor_[3].voltage * 1000));
	*(int16_t *)&tx[2] = ((int16_t)(motor_[4].voltage * 1000));
	*(int16_t *)&tx[4] = ((int16_t)(motor_[5].voltage * 1000));
	sendframe(0x200, tx);
}

void motor::update()
{
    for(auto& m : motor_)
    {
        m.fdupdate();
        m.voltupdate();
    }
}


void motor::fdupdate()
{
	this->angle = (*(int32_t *)&rx[0] / 1000.0f - this->offsetAngle) * this->dir;
	this->speed = (*(int16_t *)&rx[4] / 10 * 2 * PI_ / 60) * this->dir;
}


float motor::calcRevVolt(float sp)
{
    return 0;
}

void motor::voltupdate()
{
    float voltage = this->torque / this->torqueRatio;
	if (this->speed >= 0)
		voltage += this->calcRevVolt(this->speed);
	else if (this->speed < 0)
		voltage -= this->calcRevVolt(-this->speed); // 这段没问题 方向最后加上 先按照标量算
	// 限幅
	if (voltage > this->maxVoltage)
		voltage = this->maxVoltage;
	else if (voltage < -this->maxVoltage)
		voltage = -this->maxVoltage;
	this->voltage = voltage * this->dir;
}


void sendframe(uint16_t id, uint8_t *tx)
{
    CAN_TxHeaderTypeDef txconf;
    uint32_t tx_mailbox;
    txconf.RTR = CAN_RTR_DATA;
    txconf.IDE = CAN_ID_STD;
    txconf.StdId = id;
    txconf.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &txconf, tx, &tx_mailbox);
}







extern "C"
{
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        // 调试：进入回调就打印
        //log_printf("CAN IRQ!\r\n");
        
        if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
        {
            HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &MidCan::getcan().rxheader, MidCan::getcan().rxbuff);
            
            uint32_t id = MidCan::getcan().rxheader.StdId;
            // 遍历查找匹配ID的电机，防止数组越界
            for(auto& m : motor_)
            {   
                if(m.id == id)
                {
                    std::copy(MidCan::getcan().rxbuff, 
                              MidCan::getcan().rxbuff + 8, 
                              m.rx);
                    break;
                }
            }
        }
    }
}