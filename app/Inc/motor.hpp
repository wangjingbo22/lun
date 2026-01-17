#pragma once
#include "main.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "wrap_can.h"
#include "middleware_can.hpp"
#include <cstdint>
#include <vector>

constexpr float PI_ = 3.14159265358979f;



class motor
{
public:
    static uint8_t tx[8];
    uint8_t rx[8];

    //@todo pid类
    uint16_t id;
    float speed;					   // rad/s
	float angle, offsetAngle;		   // rad
	float voltage, maxVoltage;		   // V
	float torque, torqueRatio;		   // Nm, voltage = torque / torqueRatio
	float dir;



public:
    motor(uint16_t id, float offsetAngle, float maxVoltage, float torqueRatio, float dir)
    {
        this->id = id;
        this->offsetAngle = offsetAngle;
        this->maxVoltage = maxVoltage;
        this->torqueRatio = torqueRatio;
        this->dir = dir;

    }

    /**
     * @brief 发送电机控制指令
     * 
     * @param tx 
     */
    static void Motorsend(uint8_t* tx);

    /**
     * @brief 计划和电机发送放在同一个任务，一直解析
     * 
     */
    static void update();


    /**
     * @brief 更新电机反馈数据
     *        电机反馈的角度好像是弧度制
     */
    void fdupdate();


    /**
     * @brief 更新电机电压，这个还挺重要，设置扭矩然后转换成电压发送出去
     * 
     */
    void voltupdate();


    /**
     * @brief 设置电机扭矩
     * 
     * @param t 
     */
    inline void setTorque(float t)
    {
        this->torque = t;
    }


    /**
     * @brief 计算反反电动势电压
     * 
     */
    float calcRevVolt(float sp);



};

extern std::vector<motor> motor_ ;

void sendframe(uint16_t id , uint8_t* tx);


extern "C"
{

    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
}