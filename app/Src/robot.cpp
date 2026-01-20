#include "robot.hpp"
#include "Dbus.h"
#include "lqr.h"
#include "motor.hpp"
#include "pid.h"
#include "portmacro.h"
#include "vmc.h"
#include <algorithm>
#include <math.h>
#include <stdexcept>
#include "task.h"
#include "arm_math.h"

// 轮子半径 (根据你的实际轮子调整, 单位: 米)
#define WHEEL_RADIUS 0.044f
TaskHandle_t MainHandle;

// 速度卡尔曼滤波器
VelocityKF_t velocityKF;

// u u_left,u_right;
// f f_left,f_right;

// LegTorque uleft,uright;

// robostate x,dx,design,err;

// LegVelocity legVleft,legVright;

// LQR_Dual_K k_;

// 腿长PID参数说明: {kp, ki, kd, error, lastError, integral, maxIntegral, output, maxOutput, deadzone, errLpfRatio}
// 
// 根据你的电机特性调参:
// - 电机最大扭矩 ~0.3 Nm，最小启动扭矩 ~0.02 Nm
// - 腿长范围 0.065~0.15m，VMC转换后力约 τ/(L*系数)
// - 输出力 F 经过 VMC 转换成关节扭矩，需要保证扭矩在合理范围
//
// Kp: 弹簧刚度，误差0.01m(1cm) → 输出力，Kp=50 → F=0.5N
// Kd: 阻尼系数，速度0.1m/s → 阻尼力，Kd=5 → F=0.5N  
// maxOutput: 最大输出力，根据电机能力，建议 3~6N
// 
// ⚠️ Kd 不要太大！太大会放大噪声导致震荡
// 建议 Kd / Kp 比例在 0.05~0.2 之间




// 当前状态
static volatile RobotJumpState current_state = STATE_NORMAL;

//加上腿长的变化
void robot::updateState()
{
    // === 接收任务通知 (非阻塞) ===
    uint32_t ulNotificationValue;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotificationValue, 0) == pdTRUE) 
    {
        if (ulNotificationValue & 0x02) { // 收到跳跃命令
             // 收到命令先进入下蹲蓄力状态，而不是直接蹬地
             current_state = STATE_JUMP_SQUAT;
        }
        if (ulNotificationValue & 0x04) { // 检测到离地
             // 只有在"蹬地"阶段检测到离地才算真离地
             // 如果在下蹲时检测到离地(因为腿主动收缩导致力变小)，那是误判，忽略之
             if(current_state == STATE_JUMP_PUSH) {
                 current_state = STATE_AIR_RETRACT;
             }
        }
        if (ulNotificationValue & 0x08) { // 检测到着地
             current_state = STATE_NORMAL; 
        }
    }
    // ===========================

    

    
    //更新目标值design
    // 机械中值偏移: 如果车往X正方向(前)跑，说明重心靠前，需要给一个负的Pitch目标让它"后仰"平衡，或者调整偏移量
    // 请根据实际情况调整这个值，范围通常在 -0.1 到 0.1 之间
    float Pitch_Offset = 0.02f; 
    design.phi = Pitch_Offset;
    design.dphi = 0.0f;
    design.x = 0.0f;
    //只是先写写，具体还没想好
    design.dx = 0.0f;
    design.theta = 0.0f;
    design.dtheta = 0.0f;

    //计算误差矩阵err
    // 注意：根据LQR参数符号，我们需要确保反馈方向正确
    // kw_theta < 0, kw_v > 0, kw_x > 0
    


    //计算控制输入u
    // 完整的LQR状态反馈: u = K * err
    
    // 参数增益调整：
    // lqr_gain: 全局增益，主要限制角度环过强导致的震荡 (保持 0.2)
    // pos_boost: 位置环增强系数 (因为原始参数计算出的位置刚度太小，无法克服摩擦)
    // vel_boost: 速度环增强系数 (增加阻尼)
    float lqr_gain = 0.2f;
    float pos_boost = 25.0f; // 放大25倍: 0.2 * 25 = 5.0. 1m误差 -> 5.0 * 0.2(lqr) = 1.0Nm
    float vel_boost = 12.0f;  // 放大8倍: 0.3 * 8 = 2.4. 1m/s -> 2.4 * 0.2 = 0.48Nm
    
    float T_common = (k_.kw_theta * err.phi + 
                      k_.kw_gyro * err.dphi + 
                      k_.kw_v * err.dx * vel_boost + 
                      k_.kw_x * err.x * pos_boost) * lqr_gain;

    u_left.T  = T_common;
    u_right.T = T_common;

    // 髋关节力矩 (Tp): 负责保持腿部姿态
    // 系数 0.04f 是将 LQR 输出缩放到电机扭矩范围
    float hip_gain = 0.04f;
    // 同样加上位置反馈
    float Tp_common = (k_.kh_theta * err.phi + 
                       k_.kh_gyro * err.dphi + 
                       k_.kh_v * err.dx + 
                       k_.kh_x * err.x ) * hip_gain;

    // === 防劈叉控制 (Leg Synchronization) ===
    // 强制让左右腿角度一致
    // 如果 左腿角度 > 右腿角度，diff > 0
    // 左腿需要减小角度(后摆)，右腿需要增大角度(前摆)
    float leg_diff = LegState_l.theta - LegState_r.theta;
    float k_sync = 0.02f; // 同步刚度，根据效果调整
    float Tp_sync = k_sync * leg_diff;

    u_left.Tp  = Tp_common + Tp_sync;
    u_right.Tp = Tp_common - Tp_sync;

    f_left.Tp = u_left.Tp;
    f_right.Tp = u_right.Tp;

    update();
    errcalc();

    //接下来是腿长控制，先使用简单的pd控制模拟弹簧阻尼系统
    robot::leglengthcontrol();
    // f_left.F = 1.0*(cmd.ch[2] - (LegState_l.L+LegState_r.L)/2.0f) + 0.1*(-(legVleft.vL + legVright.vL)/2.0f);
    // f_right.F = 1.0*(cmd.ch[2] - (LegState_l.L+LegState_r.L)/2.0f) + 0.1*(-(legVleft.vL + legVright.vL)/2.0f);

    //将扭矩N*m转化成力
    float Fx_virt_L = u_left.Tp / LegState_l.L;
    float Fx_virt_R = u_right.Tp / LegState_r.L;

    uleft = GetTorque(LegState_l, motor_[1].angle, motor_[0].angle, Fx_virt_L, f_left.F);
    uright = GetTorque(LegState_r, motor_[4].angle, motor_[3].angle, Fx_virt_R, f_right.F);

    // 关节电机使用 VMC 转换后的扭矩
    motor_[0].setTorque(-uleft.tau4);   // 左腿后关节
    motor_[1].setTorque(-uleft.tau1);   // 左腿前关节
    motor_[3].setTorque(-uright.tau4);  // 右腿后关节
    motor_[4].setTorque(-uright.tau1);  // 右腿前关节
    // 驱动轮使用 LQR 计算的扭矩
    motor_[2].setTorque(u_left.T);
    motor_[5].setTorque(u_right.T);
    
    // 安全保护：如果倾角过大，关闭电机防止乱撞
    if (fabsf(ins.Pitch) > 0.5f) { // 约 28 度
        for(auto& m : motor_) m.setTorque(0);
    }
}

/**
* @brief 最短腿长0.0627，最长腿长0.1501

大概就在0.065和0.145之间

遥控器数值0-660

最快腿长变化速度0.5m/s

20-330设置成0.3m/s的速度变大

330-660设置为0.5m/s的速度变大

对期望速度积分得到期望腿长，限制在0.065和0.145之间

R标正对是前方，向上pitch增大，向右倾roll增大，所以向右倾得时候左腿变短，右腿变长适应倾斜地面
 * 
 */
void robot::leglengthcontrol()
{
    // ============================================================
    // 逻辑分支：根据当前状态选择不同的控制策略
    // ============================================================
    
    if (current_state != STATE_NORMAL) 
    {
        // >>>>> 跳跃模式逻辑 (接收到 0x02 通知后进入) <<<<<

        // 1. 下蹲蓄力阶段 (Squat)
        if (current_state == STATE_JUMP_SQUAT)
        {
            float Target_Squat_L = 0.065f; // 下蹲目标：蹲到 0.065m
            float Ks = 300.0f; 
            float Kd = 8.0f;
            
            // 使用PID控制下蹲
            f_left.F  = Ks * (Target_Squat_L - LegState_l.L) - Kd * legVleft.vL;
            f_right.F = Ks * (Target_Squat_L - LegState_r.L) - Kd * legVright.vL;

            // 检查：是否已经蹲到位了？
            // 如果腿长小于 0.07m，说明已经蹲下去了，立即切换到蹬地状态
            if (LegState_l.L < 0.07f && LegState_r.L < 0.07f)
            {
                current_state = STATE_JUMP_PUSH;
            }
        }
        // 2. 爆发蹬地阶段 (Push)
        else if (current_state == STATE_JUMP_PUSH) 
        {
            // 开环给定最大推力，无视PID
            f_left.F = 180.0f; 
            f_right.F = 180.0f;
        }
        // 3. 空中收腿阶段 (Air)
        else if (current_state == STATE_AIR_RETRACT)
        {
            // 闭环位置控制缩腿
            // 目标设为 0.08m (比正常短)，防止脚尖磕地，也为落地缓冲留出行程
            float Target_L = 0.08f; 
            float Ks = 200.0f; // 空中不需要太硬
            float Kd = 6.0f;   // 适当阻尼
            
            f_left.F  = Ks * (Target_L - LegState_l.L) - Kd * legVleft.vL;
            f_right.F = Ks * (Target_L - LegState_r.L) - Kd * legVright.vL;
        }
    }
    else 
    {
        // >>>>> 正常平衡逻辑 (默认) <<<<<
        
        float LEG_L0 = 0.1f; 
        
        // 1. 物理参数
        float Ks = 300.0f; 
        float Kd = 12.0f;
        float FeedForward = 8.0f; 

        // 2. 计算误差
        float err_L = LEG_L0 - LegState_l.L;
        float err_R = LEG_L0 - LegState_r.L;

        // 3. 计算垂直力 Fy (牛顿)
        f_left.F  = Ks * err_L - Kd * legVleft.vL + FeedForward;
        f_right.F = Ks * err_R - Kd * legVright.vL + FeedForward;
    }

    // ============================================================
    // 统一安全限幅 (无论什么模式都必须遵守物理极限)
    // ============================================================


    f_left.F = std::clamp(f_left.F, 0.0f, F_MAX);
    f_right.F = std::clamp(f_right.F, 0.0f, F_MAX);

    //固定模式
    // PID_SingleCalc(&leglengthleft,0.1f,LegState_l.L);
    // PID_SingleCalc(&leglengthright,0.1f,LegState_r.L);

    // f_left.F = leglengthleft.output;
    // f_right.F = leglengthright.output;    

    //变长模式
    // float dL = 0.0f;
    // float designL = 0.0f;
    // if(cmd.ch[2] >=20 && cmd.ch[2] <330)
    //     dL = 0.3f;
    // else if(cmd.ch[2] >=330 && cmd.ch[2] <=660)
    //     dL = 0.5f;
    // else if (cmd.ch[2] <= -20 && cmd.ch[2] > -330)
    //     dL = -0.3f;
    // else if (cmd.ch[2] < -330)
    //     dL = -0.5f;


    // designL += dL * 0.001f;

    // PID_SingleCalc(&leglengthleft,designL,LegState_l.L);
    // PID_SingleCalc(&leglengthright,designL,LegState_r.L);

    // //如果roll为正，需要增长右腿，减小左腿，pid算出的结果是负的，应该加到左腿上，减到右腿上
    // //如果roll为负，需要增长左腿，减小右腿，pid算出的结果是正的，应该加到左腿上，减到右腿上
    // PID_SingleCalc(&Roll, 0.0f, ins.Roll);

    // f_left.F = leglengthleft.output + Roll.output;
    // f_right.F = leglengthright.output - Roll.output;
}



float* robot::DSP_LQR_Calculation(void)
{
    // -----------------------------------------------------------
    // 1. 准备矩阵数据缓存
    // -----------------------------------------------------------
    // K: 2行6列 (输入)
    // X: 6行1列 (状态)
    // U: 2行1列 (输出)
    float32_t K_data[12];
    float32_t X_data[6];
    float32_t U_data[2];

    // -----------------------------------------------------------
    // 2. 填充 K 矩阵 (Row-Major Order)
    // -----------------------------------------------------------
    // Row 0: 轮子电机反馈增益 (Wheel Motor Gains)
    K_data[0] = k_.kw_theta;
    K_data[1] = k_.kw_gyro;
    K_data[2] = k_.kw_x;
    K_data[3] = k_.kw_v;
    K_data[4] = k_.kw_phi;
    K_data[5] = k_.kw_phi_v;

    // Row 1: 髋关节反馈增益 (Hip/Body Gains)
    K_data[6] = k_.kh_theta;
    K_data[7] = k_.kh_gyro;
    K_data[8] = k_.kh_x;
    K_data[9] = k_.kh_v;
    K_data[10] = k_.kh_phi;
    K_data[11] = k_.kh_phi_v;

    // -----------------------------------------------------------
    // 3. 填充状态向量 X (顺序必须与 K 的列定义一致)
    // -----------------------------------------------------------
    // 基于 robot.cpp 逻辑的映射:
    // Col 0 (Theta Coeff) -> err.theta (摆杆/腿角度)
    // Col 1 (Gyro Coeff)  -> err.dtheta(摆杆/腿角速度)
    // Col 2 (X Coeff)     -> err.x     (位移)
    // Col 3 (V Coeff)     -> err.dx    (速度)
    // Col 4 (Phi Coeff)   -> err.phi   (机体倾角)
    // Col 5 (PhiV Coeff)  -> err.dphi  (机体角速度)
    
    // 注意: 这里不包含原始代码中的 pos_boost/vel_boost 等额外增益系数
    // 仅计算纯粹的 LQR: u = K * x
    X_data[0] = err.theta;
    X_data[1] = err.dtheta;
    X_data[2] = err.x;
    X_data[3] = err.dx;
    X_data[4] = err.phi;
    X_data[5] = err.dphi;

    // -----------------------------------------------------------
    // 4. 初始化矩阵对象
    // -----------------------------------------------------------
    arm_matrix_instance_f32 matK;
    arm_matrix_instance_f32 matX;
    arm_matrix_instance_f32 matU;

    // 初始化特定维度的矩阵
    arm_mat_init_f32(&matK, 2, 6, K_data);
    arm_mat_init_f32(&matX, 6, 1, X_data);
    arm_mat_init_f32(&matU, 2, 1, U_data);

    // -----------------------------------------------------------
    // 5. 执行矩阵乘法: U = K * X
    // -----------------------------------------------------------
    // 结果存储在 U_data 中
    arm_mat_mult_f32(&matK, &matX, &matU);

    // -----------------------------------------------------------
    // 6. 提取结果
    // -----------------------------------------------------------
    // 这里的 T 和 Tp 是纯粹基于 LQR K 矩阵计算得到的
    float T_matrix  = U_data[0]; 
    float Tp_matrix = U_data[1];

    return U_data;
}
 
void robot::update()
{
    //更新左右腿位置
    LegState_l = GetState(motor_[1].angle, motor_[0].angle);
    LegState_r = GetState(motor_[4].angle, motor_[3].angle);
    //更新左右腿速度
    legVleft = GetVelocity(motor_[1].angle, motor_[0].angle, motor_[1].speed, motor_[0].speed);
    legVright = GetVelocity(motor_[4].angle, motor_[3].angle, motor_[4].speed, motor_[3].speed);

    //更新反馈矩阵K

    k_ = Get_LQR_Dual_K((LegState_l.L+LegState_r.L)/2.0f);

    //================== 卡尔曼滤波融合速度 ==================//
    // 1. 获取编码器速度 (轮子角速度 * 轮子半径 = 线速度)
    //float encoder_velocity = (motor_[2].speed + motor_[5].speed) / 2.0f * WHEEL_RADIUS;

    // 2. 获取IMU加速度 (使用去除重力后的运动加速度, 取X轴前进方向)
    //    注意: ins.MotionAccel_b 是机体系下的运动加速度
    //    根据你的坐标系定义选择正确的轴
    //    X轴加速度, 根据实际调整
    
    // 3. 卡尔曼滤波更新
    filtered_dx = VelocityKF_Update(&velocityKF, (motor_[2].speed + motor_[5].speed) / 2.0f * WHEEL_RADIUS, ins.MotionAccel_b[0]);
    if(fabsf(filtered_dx) < 0.05f) filtered_dx = 0.0f;
    //=========================================================//

    //更新机器人状态向量x
    x.phi = ins.Pitch;
    x.dphi = -ins.Gyro[1];
    x.x = (motor_[2].angle + motor_[5].angle) / 2.0f * WHEEL_RADIUS;  // 位移也换算为米
    // 使用卡尔曼滤波后的速度
    x.dx = filtered_dx;
    x.theta =(LegState_l.theta + LegState_r.theta) / 2.0f - M_PI_2- ins.Pitch ;
    x.dtheta = (legVleft.vPhi + legVright.vPhi) / 2.0f;

    design.dx = DbusData.ch[1]; //暂时这样
}

void robot::errcalc()
{
    // 1. 角度误差 (Current - Target)
    // kw_theta < 0. 若 phi > 0 (后仰), u = K*phi < 0 (后向扭矩) -> 机体前倾 -> 恢复.
    err.phi    =  x.phi - design.phi;
    err.dphi   =  x.dphi - design.dphi;
    
    if(fabsf(x.phi) < 0.4f) { 
        err.x  =   x.x - design.x;
    } else {
        err.x  =   0.0f; // 倒地后不积分位置
    }
    err.x = std::clamp(err.x, -1.0f, 1.0f);
    
    err.dx     =  x.dx - design.dx;
    err.theta  =  x.theta;
    err.dtheta =  x.dtheta;
}

float robot::legharmony()
{
    float theta_diff = LegState_l.theta - LegState_r.theta;
    float kp_harmony = 0.05f; // 和同步增益类似，但更温和
    float kd_harmony = 0.01f;

    float harmony_torque = kp_harmony * theta_diff + kd_harmony * (legVleft.vPhi - legVright.vPhi);

    return harmony_torque;
}

