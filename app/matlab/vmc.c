/**
 * @file vmc.c
 * @author 使用gemini3
 * @brief 五杆机构虚拟模型控制（VMC）核心算法
 * @version 1.0
 * @date 2025-12-03
 *
 */
/* Five-Bar VMC Control */
/* Params: l1=0.05, l2=0.105, d=0.06 */
#include <math.h>
#include <stdio.h>

#include "vmc.h"

/* 
暂时定义两个全局变量测试一下，以后再优化一下
*/

LegState LegState_l;
LegState LegState_r;
LegTorque LegTorque_;

// ===========================================================
// 函数 1：根据编码器角度解算当前状态
// ===========================================================
LegState GetState(float phi1, float phi4) {
    LegState s;
    
    // 1. 预计算三角函数
    float c1 = cosf(phi1); float s1 = sinf(phi1);
    float c4 = cosf(phi4); float s4 = sinf(phi4);

    // 2. 计算膝盖坐标 (l1=0.05, d=0.06)
    // xB = -0.03 + 0.05*c1, yB = 0.05*s1
    // xD =  0.03 + 0.05*c4, yD = 0.05*s4
    
    // 3. 计算 BD 向量
    // dx = xD - xB
    float dx = 0.06f + 0.05f * (c4 - c1);
    float dy = 0.05f * (s4 - s1);
    
    float dist2 = dx*dx + dy*dy;
    
    // 4. 几何解算 (l2=0.105)
    // h^2 = l2^2 - (dist/2)^2 = 0.011025 - dist2/4
    float h_sq = 0.011025f - dist2 * 0.25f;

    if (h_sq < 0.0f) {
        s.valid = 0; s.x=0; s.y=0; s.L=0; s.theta=0;
        return s;
    }
    float h = sqrtf(h_sq);
    
    // 5. 计算末端 C (利用对称性中点公式)
    // xM = (xB + xD)/2
    float xM = 0.025f * (c1 + c4); // -0.03和+0.03抵消了
    float yM = 0.025f * (s1 + s4);
    
    // 优化：inv_dist = 1/dist
    float inv_dist = 1.0f / sqrtf(dist2);
    
    // C 点坐标 (左手定则/膝盖向上)
    // x = xM - h * dy / dist
    // y = yM + h * dx / dist
    s.x = xM - h * dy * inv_dist;
    s.y = yM + h * dx * inv_dist;
    
    s.L = sqrtf(s.x*s.x + s.y*s.y);
    s.theta = atan2f(s.y, s.x);
    s.valid = 1;
    
    return s;
}

// ===========================================================
// 函数 2：根据期望力(Fx,Fy)计算电机扭矩(tau1,tau4)
// ===========================================================
LegTorque GetTorque(LegState s, float phi1, float phi4, float Fx, float Fy) {
    LegTorque t;
    if (!s.valid) { t.tau1=0; t.tau4=0; return t; }

    // 重新获取必要的变量
    float x = s.x;
    float y = s.y;
    float c1 = cosf(phi1); float s1 = sinf(phi1);
    float c4 = cosf(phi4); float s4 = sinf(phi4);

    // 1. 计算虚拟连杆向量 (r_CB, r_CD)
    // B = (-0.03 + 0.05c1, 0.05s1)
    float rx_CB = x - (-0.03f + 0.05f * c1);
    float ry_CB = y - (0.05f * s1);
    
    // D = ( 0.03 + 0.05c4, 0.05s4)
    float rx_CD = x - ( 0.03f + 0.05f * c4);
    float ry_CD = y - ( 0.05f * s4);

    // 2. 雅可比行列式 (Determinant)
    float det = rx_CB * ry_CD - rx_CD * ry_CB;
    
    if (fabsf(det) < 1e-6f) { t.tau1=0; t.tau4=0; return t; } // 奇异点保护
    float inv_det = 1.0f / det;

    // 3. 驱动力臂投影系数
    // k1 = l1 * (sin(q1)*rx - cos(q1)*ry)
    float k1 = 0.05f * (s1 * rx_CB - c1 * ry_CB);
    float k2 = 0.05f * (s4 * rx_CD - c4 * ry_CD);

    // 4. 最终扭矩映射 (J^T * F)
    t.tau1 = (k1 * inv_det) * (ry_CD * Fx - rx_CD * Fy);
    t.tau4 = (k2 * inv_det) * (-ry_CB * Fx + rx_CB * Fy);

    return t;
}

LegVelocity GetVelocity(float phi1, float phi4, float dphi1, float dphi4) {
    LegVelocity v;
    
    // 1. 预计算三角函数
    float c1 = cosf(phi1); float s1 = sinf(phi1);
    float c4 = cosf(phi4); float s4 = sinf(phi4);

    // ---------------------------------------------------------
    // MATLAB 自动生成的导数公式 (已针对 l1=0.05, l2=0.105 优化)
    // ---------------------------------------------------------
    
    // 辅助变量 (几何项)
    float t2 = c1 * 0.05f;
    float t3 = t2 - 0.03f;  // xB部分
    float t4 = c4 * 0.05f;
    float t5 = t4 + 0.03f;  // xD部分
    float t6 = t3 - t5;     // dx
    float t7 = s1 * 0.05f;
    float t8 = s4 * 0.05f;
    float t9 = t7 - t8;     // -dy
    float t10 = t6 * t6;
    float t11 = t9 * t9;
    float t12 = t10 + t11;  // dist^2
    float t13 = sqrtf(t12); // dist
    float t14 = 1.0f / t13;
    
    // h 相关
    float t15 = t12 * 0.5f;
    float t16 = 1.0f / t12;
    float t17 = sqrtf(-t15 * t15 * t16 + 0.011025f); // h
    
    // 速度映射相关中间量
    float t18 = t15 * t16 * t6;
    float t19 = t3 - t18;
    float t20 = t9 * t17 * t14;
    float t21 = t19 + t20; // -xC
    float t22 = t15 * t16 * t9;
    float t23 = t7 - t22;
    float t24 = t6 * t17 * t14;
    float t25 = t23 - t24; // yC
    float t26 = t21 * t21;
    float t27 = t25 * t25;
    float t28 = t26 + t27; // L^2
    float t29 = 1.0f / sqrtf(t28); // 1/L
    
    // 关节速度项
    float t30 = dphi1 * s1 * 0.05f;
    float t31 = dphi4 * s4 * 0.05f;
    float t32 = t30 - t31;
    float t33 = dphi1 * c1 * 0.05f;
    float t34 = dphi4 * c4 * 0.05f;
    float t35 = t33 - t34;
    
    // 复杂的链式求导项 (由 MATLAB 生成)
    float t36 = t15 * t16 * t32;
    float t37 = t15 * t16 * t35;
    float t38 = 1.0f / t17;
    float t39 = t15 * t16 * t17 * t32 * 2.0f;
    float t40 = t15 * t16 * t17 * t35 * 2.0f;
    float t41 = t32 * t9 * 2.0f;
    float t42 = t6 * t35 * 2.0f;
    float t43 = t41 + t42;
    float t44 = t15 * t16 * t43;
    float t45 = t39 + t40 - t44;
    float t46 = t38 * t45 * 0.5f;
    float t47 = t20 + t32 - t36;
    float t48 = t9 * t14 * t46;
    float t49 = t9 * t17 * t32 * t16;
    float t50 = t9 * t17 * t35 * t16;
    float t51 = t17 * t35 * t14;
    float t52 = t47 - t48 + t49 + t50 + t51;
    float t53 = t24 - t35 + t37;
    float t54 = t6 * t14 * t46;
    float t55 = t6 * t17 * t32 * t16;
    float t56 = t6 * t17 * t35 * t16;
    float t57 = t17 * t32 * t14;
    float t58 = t53 + t54 - t55 - t56 + t57;

    // ---------------------------------------------------------
    // 最终输出: vL 和 vPhi
    // ---------------------------------------------------------
    
    // vL = (x*vx + y*vy) / L
    // 注意: t21是-x, t25是y, t52是-vx, t58是vy
    v.vL = t29 * (t21 * t52 + t25 * t58);
    
    // vPhi = (x*vy - y*vx) / L^2
    v.vPhi = (1.0f / t28) * (-t21 * t58 - t25 * t52);
    
    return v;
}


/**
 * @brief 下面的代码是matlab生成的，跟ai手写的代码效率没啥不同，就看着玩玩
 * 
 */
/* 
Five-Bar VMC Control 
Params: l1=0.05, l2=0.105, d=0.06 
#include <math.h>
#include <stdio.h>

typedef struct {
    float x, y, L, phi;
    int valid;
} LegState;

typedef struct {
    float tau1, tau4;
} LegTorque;

LegState GetState(float phi1, float phi4) {
    LegState s;
    float t0;
    // Pre-compute trig
    float c1 = cosf(phi1); float s1 = sinf(phi1);
    float c4 = cosf(phi4); float s4 = sinf(phi4);

    float x =   cos(c1)/4.0E+1+c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0;
    float y =   sin(c1)/4.0E+1+s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0;

    s.x = x;
    s.y = y;
    s.L = sqrtf(x*x + y*y);
    s.phi = atan2f(y, x);
    // Valid if leg length is within range (0.02m ~ 0.15m)
    s.valid = (s.L > 0.02f && s.L < 0.15f) ? 1 : 0;
    return s;
}

LegTorque GetTorque(LegState s, float phi1, float phi4, float Fx, float Fy) {
    LegTorque t;
    float t0;
    if (!s.valid) { t.tau1=0; t.tau4=0; return t; }

    float x = s.x;
    float y = s.y;
    float c1 = cosf(phi1); float s1 = sinf(phi1);
    float c4 = cosf(phi4); float s4 = sinf(phi4);

    t.tau1 =   (sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(Fx*((sin(c1)*(cos(c1)*(-1.0/4.0E+1)+c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0+3.0/1.0E+2))/2.0E+1-(cos(c1)*(sin(c1)*(-1.0/4.0E+1)+s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0))/2.0E+1)*(sin(c1)/4.0E+1-s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0)-Fy*((sin(c1)*(cos(c1)*(-1.0/4.0E+1)+c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0+3.0/1.0E+2))/2.0E+1-(cos(c1)*(sin(c1)*(-1.0/4.0E+1)+s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0))/2.0E+1)*(cos(c1)/4.0E+1-c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0-3.0/1.0E+2))*1.0/sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2)*-2.0E+4)/(cos(c1-phi4)*5.0E+1+cos(c1)*6.0E+1-c4*6.0E+1-8.6E+1);
    t.tau4 =   (sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(Fx*((s4*(cos(c1)/4.0E+1-c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0-3.0/1.0E+2))/2.0E+1-(c4*(sin(c1)/4.0E+1-s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0))/2.0E+1)*(sin(c1)*(-1.0/4.0E+1)+s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0)-Fy*((s4*(cos(c1)/4.0E+1-c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0-3.0/1.0E+2))/2.0E+1-(c4*(sin(c1)/4.0E+1-s4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(cos(c1)*(-1.0/2.0E+1)+c4/2.0E+1+3.0/5.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0))/2.0E+1)*(cos(c1)*(-1.0/4.0E+1)+c4/4.0E+1+(1.0/sqrtf(powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)+powf(sin(c1)-s4,2.0)*2.5E+1)*(sin(c1)/2.0E+1-s4/2.0E+1)*sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2))/2.0+3.0/1.0E+2))*1.0/sqrtf(-powf(cos(c1)*-5.0+c4*5.0+6.0,2.0)-powf(sin(c1)-s4,2.0)*2.5E+1+4.41E+2)*2.0E+4)/(cos(c1-phi4)*5.0E+1+cos(c1)*6.0E+1-c4*6.0E+1-8.6E+1);
    return t;
}


*/