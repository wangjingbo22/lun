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
#include "FreeRTOS.h"
#include "task.h"


// #define M_PI_2 1.57079632679489661923f
constexpr float F_MAX = 15.0f;  // HT4315: 2电机×0.65Nm / 0.1m ≈ 13N (留余量取15N)

extern TaskHandle_t MainHandle;

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

// 定义机器人状态枚举
enum RobotJumpState {
    STATE_NORMAL = 0,     // 正常平衡
    STATE_JUMP_SQUAT,     // 1. 蓄力下蹲 (新增)
    STATE_JUMP_PUSH,      // 2. 爆发蹬地
    STATE_AIR_RETRACT,    // 3. 空中缩腿
    STATE_LANDING_RECOVERY // 4. 着地缓冲 (暂未特殊处理，自动回Normal)
};

// extern u u_left,u_right;
// extern f f_left,f_right;

// extern LegTorque uleft,uright;

// extern  robostate x,dx,design,err;

// extern LegVelocity legVleft,legVright;

// extern LQR_Dual_K k_;
// extern PID leglength;
// extern PID Roll;

// // 速度卡尔曼滤波器
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

TaskHandle_t MainHandle;

//u是控制输入，f是腿部作用力
u u_left,u_right;
f f_left,f_right;

LegTorque uleft,uright;

robostate x,dx,design,err;

LegVelocity legVleft,legVright;

LQR_Dual_K k_;

// PID leglengthleft  = {60.0f, 0.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f, 0.002f, 0.6f};
// PID leglengthright = {60.0f, 0.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f, 0.002f, 0.6f};

// // Roll补偿: 根据倾斜角度调整左右腿长差异
// // Kp=2: 倾斜1rad(57°) → 补偿力2N (实际倾斜很小，所以这个增益要适中)
// PID Roll = {2.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5f, 0.02f, 0.3f};

// 速度卡尔曼滤波器

//==================================成员函数================================//

/**
 * @brief pitch超过一定角度力设为0
 * 
 */
void forsafe();

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

void update();

void errcalc();

void extracted();

void control();

/**
 * @brief 使用DSP库进行矩阵运算，根据当前误差 err 和增益 k_ 计算 T 和 Tp
 *        函数结果存储在静态变量中，可用于调试或替换原有的标量结算
 */
void DSP_LQR_Calculation(void);

float legharmony();

void setTorque();

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