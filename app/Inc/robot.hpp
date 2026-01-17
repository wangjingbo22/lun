#pragma once

#include "ins_task.h"
#include "lqr.h"
#include "pid.h"
#include "vmc.h"
#include "main.h"
#include "QuaternionEKF.h"
#include "motor.hpp"
#include <vector>
#include "Dbus.h"
#include "math.h"
#include "velocity_kalman.h"


// #define M_PI_2 1.57079632679489661923f



typedef struct
{
    float theta;
    float dtheta;
    float x;
    float dx;
    float phi;
    float dphi;
}robostate;

typedef struct
{
    float T;
    float Tp;
}u;

typedef struct
{
    float F;
    float Tp;
} f;

extern u u_left,u_right;
extern f f_left,f_right;

extern LegTorque uleft,uright;

extern  robostate x,dx,design,err;

extern LegVelocity legVleft,legVright;

extern LQR_Dual_K k_;
extern PID leglength;
extern PID Roll;

// 速度卡尔曼滤波器
extern VelocityKF_t velocityKF;

class robot
{
public:
//==================================成员变量================================//

INS_t& ins = INS;
// std::vector<motor> motor_;
Dbus_t& cmd = DbusData;

// 滤波后的速度
float filtered_dx = 0.0f;




//==================================成员函数================================//


/**
 * @brief 控制腿长，使用简单的PD控制模拟弹簧阻尼系统
 * 
 */
void leglengthcontrol();


/**
 * @brief 更新机器人的姿态，即状态向量x和输入向量u
 * 
 */
void updateState();

//==================================单例模式函数================================//
inline static robot& getInstance()
{
    static robot instance;
    return instance;
}
robot(const robot&) = delete;
robot& operator=(const robot&) = delete;
//=========================================================================//
private:
    robot(){} 
};