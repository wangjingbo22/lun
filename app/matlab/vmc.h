#ifndef VMC_H
#define VMC_H

#include "math.h"
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    float x, y, L, theta;
    int valid;
} LegState;

typedef struct {
    float tau1, tau4;
} LegTorque;

typedef struct {
    float vL;   // 腿长伸缩速度 (m/s)
    float vPhi; // 腿摆动角速度 (rad/s)
} LegVelocity;

extern LegState LegState_l;
extern LegState LegState_r;
extern LegTorque LegTorque_;


// ===========================================================
// 函数 1：根据编码器角度解算当前状态
// ===========================================================
LegState GetState(float phi1, float phi4);


// ===========================================================
// 函数 2：根据期望力(Fx,Fy)计算电机扭矩(tau1,tau4)
// ===========================================================
LegTorque GetTorque(LegState s, float phi1, float phi4, float Fx, float Fy);


// ============================================================
// 函数3: 计算末端状态微分
// 输入: 
//   phi1, phi4: 当前关节角度 (rad)
//   dphi1, dphi4: 当前关节角速度 (rad/s)
// 输出:
//   vL, vPhi: 虚拟腿的速度
// ============================================================
LegVelocity GetVelocity(float phi1, float phi4, float dphi1, float dphi4);


#ifdef __cplusplus
}
#endif
#endif // VMC_H