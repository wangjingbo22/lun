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

// ============================================================
// HT4315 电机力矩转换系数
// τ_actual = 1.36 × torque_code
// 所以: torque_code = τ_lqr / 1.36 = τ_lqr × 0.735
// ============================================================
#define TORQUE_TO_CODE  0.735f   // LQR输出(N·m) → 电机控制量

TaskHandle_t MainHandle;

// 速度卡尔曼滤波器
VelocityKF_t velocityKF;
// 腿长PID参数说明: {kp, ki, kd, error, lastError, integral, maxIntegral, output, maxOutput, deadzone, errLpfRatio}
// 
// 根据你的电机特性调参:
// - 电机最大扭矩 ~0.3 Nm，最小启动扭矩 ~0.02 Nm
// - 腿长范围 0.065~0.15m，VMC转换后力约 τ/(L*系数)
// - 输出力 F 经过 VMC 转换成关节扭矩，需要保证扭矩在合理范围
// Kp: 弹簧刚度，误差0.01m(1cm) → 输出力，Kp=50 → F=0.5N
// Kd: 阻尼系数，速度0.1m/s → 阻尼力，Kd=5 → F=0.5N  
// maxOutput: 最大输出力，根据电机能力，建议 3~6N





// 当前状态
static volatile RobotJumpState current_state = STATE_NORMAL;


void robot::control() {
  // === 接收任务通知 (非阻塞) ===
  extracted();
  // ===========================
  update();
  errcalc();
  DSP_LQR_Calculation();
  // 接下来是腿长控制，先使用简单的pd控制模拟弹簧阻尼系统
  leglengthcontrol();
  
  // ============================================================
  // VMC: 将足端力 (Fx, Fy) 转换为关节扭矩 (tau1, tau4)
  // ============================================================
  // Fx = Tp / L  (髋关节力矩转换为水平力)
  // Fy = F       (腿长控制的垂直力)
  // ============================================================
  
  // 启用髋关节力矩 Tp 控制
  float hip_gain = 0.02f;  // 从很小的值开始，帮助平衡
  float Fx_left  = u_left.Tp  * hip_gain / LegState_l.L;
  float Fx_right = u_right.Tp * hip_gain / LegState_r.L;
  
  // ============================================================
  // 腿同步补偿：防止两腿劈叉
  // ============================================================
  float theta_diff = LegState_l.theta - LegState_r.theta;  // 左腿 - 右腿
  float dtheta_diff = legVleft.vPhi - legVright.vPhi;      // 角速度差
  
  float Kp_sync = 4.0f;    // 同步刚度 (加大)
  float Kd_sync = 2.0f;    // 同步阻尼
  
  // 同步水平力：左腿角度大于右腿时，给左腿负Fx(向后)，右腿正Fx(向前)
  float Fx_sync = Kp_sync * theta_diff + Kd_sync * dtheta_diff;
  
  Fx_left  += Fx_sync;   // 左腿减去同步力
  Fx_right -= Fx_sync;   // 右腿加上同步力
  
  uleft = GetTorque(LegState_l, motor_[1].angle, motor_[0].angle, Fx_left, f_left.F);
  uright = GetTorque(LegState_r, motor_[4].angle, motor_[3].angle, Fx_right, f_right.F);
  
  // 关节电机使用 VMC 转换后的速度误差 (err.dx) 分别乘以了 25.0 和 12.扭矩
  setTorque();
  // 安全保护：如果倾角过大，关闭电机防止乱撞
  forsafe();
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
            float Ks = 150.0f;  // 下蹲时刚度稍大，快速到位
            float Kd = 12.0f;
            
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
            // 开环给定最大推力 (受F_MAX=15N限制)
            // HT4315峰值: 2*0.8/0.1 = 16N, 取15N
            f_left.F = 15.0f; 
            f_right.F = 15.0f;
        }
        // 3. 空中收腿阶段 (Air)
        else if (current_state == STATE_AIR_RETRACT)
        {
            // 闭环位置控制缩腿
            // 目标设为 0.08m (比正常短)，防止脚尖磕地，也为落地缓冲留出行程
            float Target_L = 0.08f; 
            float Ks = 100.0f;  // 空中不需要太硬
            float Kd = 10.0f;   // 适当阻尼
            
            f_left.F  = Ks * (Target_L - LegState_l.L) - Kd * legVleft.vL - legharmony();
            f_right.F = Ks * (Target_L - LegState_r.L) - Kd * legVright.vL + legharmony();
        }
    }
    else 
    {
        // >>>>> 正常平衡逻辑 (默认) <<<<<
        
        float LEG_L0 = 0.1f; 
        
        // ============================================================
        // HT4315 电机参数计算的 PD 参数
        // 额定扭矩: 0.65 N·m, 堵转扭矩: 0.8 N·m
        // 每腿2电机, L=0.1m, F_max ≈ 2*0.65/0.1 = 13N
        // ============================================================
        
        // Ks (弹簧刚度): 用更小的值
        float Ks = 60.0f;    // 误差1cm -> 0.6N
        
        // Kd (阻尼系数): 恢复较小值
        float Kd = 10.0f;    // 速度0.1m/s -> 1N 阻尼力
        
        // 重力前馈: 机体质量约1.4kg, 重力 ≈ 14N, 双腿分担
        float FeedForward = 7.0f;  // 每条腿承担约7N重力

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



void robot::DSP_LQR_Calculation(void)
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
    // 轮子增益缩放系数 (震荡时调小，响应慢时调大)
    // -----------------------------------------------------------
    float wheel_gain = 0.135f;   // 整体增益
    float phi_boost  = 3.0f;     // phi增益放大
    float x_boost    = 2.16f;     // 位置增益 (减小防止振荡发散)
    float dx_boost   = 2.36f;     // 速度增益 (增大提供阻尼!)

    // -----------------------------------------------------------
    // 2. 填充 K 矩阵 (Row-Major Order)
    // -----------------------------------------------------------
    // 根据 lqr_1_20.m 定义:
    // state_vars = [theta(摆杆), d_theta, x, d_x, phi(机体), d_phi]
    
    // Row 0: 轮子电机反馈增益 (乘以缩放系数)
    K_data[0] = k_.kw_theta * wheel_gain;   // Col 0: Theta (摆杆/腿)
    K_data[1] = k_.kw_gyro  * wheel_gain;   // Col 1: dTheta
    K_data[2] = k_.kw_x     * wheel_gain * x_boost;    // Col 2: X (位置)
    K_data[3] = k_.kw_v     * wheel_gain * dx_boost;   // Col 3: dX (速度阻尼!)
    K_data[4] = k_.kw_phi   * wheel_gain * phi_boost;  // Col 4: Phi
    K_data[5] = k_.kw_phi_v * wheel_gain * phi_boost;  // Col 5: dPhi

    // Row 1: 髋关节反馈增益
    K_data[6] = k_.kh_theta;
    K_data[7] = k_.kh_gyro;
    K_data[8] = k_.kh_x;
    K_data[9] = k_.kh_v;// * hip_gain;
    K_data[10]= k_.kh_phi;
    K_data[11]= k_.kh_phi_v;

    // -----------------------------------------------------------
    // 3. 填充状态向量 X (顺序必须与 K 的列定义一致)
    // -----------------------------------------------------------
    // 映射关系确立:
    // Theta (Matlab) -> err.theta (C++, 腿角度)
    // Phi   (Matlab) -> err.phi   (C++, 机体角度)
    
    X_data[0] = err.theta;   // Index 0: Theta (摆杆/腿)
    X_data[1] = err.dtheta;
    X_data[2] = err.x;       // Index 2: X
    X_data[3] = err.dx;
    X_data[4] = err.phi;     // Index 4: Phi (机体)
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
    // LQR 输出力矩 (N·m)
    // 注意：没有遥控器输入时，cmd.ch[2] 应该为0
    float T_lqr = U_data[0];
    float Tp_lqr = U_data[1];
    
    // 遥控器差速转向 (如果没有遥控器，确保 cmd.ch[2] = 0)
    u_left.T = T_lqr + cmd.ch[2];
    u_right.T = T_lqr - cmd.ch[2];
    u_left.Tp = Tp_lqr;
    u_right.Tp = Tp_lqr;
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
    x.dtheta = (legVleft.vPhi + legVright.vPhi) / 2.0f - ins.Gyro[1];

    // 遥控器速度指令，加死区防止漂移
    float raw_cmd = DbusData.ch[1];
    if(fabsf(raw_cmd) < 10.0f) {  // 死区
        design.dx = 0.0f;
    } else {
        design.dx = raw_cmd * 0.001f;  // 缩放到合理范围
    }
}

void robot::errcalc()
{
    // ============================================================
    // 误差计算: err = current - target
    // ============================================================
    // 注意：如果机器人总是向某一边倒，检查以下几点：
    // 1. ins.Pitch 零点是否准确（静止时应为0）
    // 2. 如果有固定偏差，在 design.phi 中补偿
    // ============================================================
    
    // 机体角度误差 (phi)
    // design.phi 可用于补偿 IMU 零点偏差
    // 如果机器人向前跑，把 design.phi 调正一点（让它后仰）
    // 如果机器人向后跑，把 design.phi 调负一点（让它前倾）
    design.phi = 0.025f;  // 约 +1.1°，让机器人稍微后仰来抵消前跑趋势
    
    // 角度死区：微小角度不响应，防止因为小角度一直跑
    float phi_deadzone = 0.01f;  // 约0.57度的死区
    float phi_error = x.phi - design.phi;
    if(fabsf(phi_error) < phi_deadzone) {
        err.phi = 0.0f;
    } else {
        // 死区外，减去死区宽度，避免跳变
        err.phi = phi_error - copysignf(phi_deadzone, phi_error);
    }
    err.dphi   = x.dphi;
    
    // 位置误差 (启用位置控制，防止跑远)
    // 只有当机体接近平衡时才累积位置误差
    if(fabsf(x.phi) < 0.3f) {
        err.x = x.x - design.x;  // design.x 初始为0，机器人会尝试回到原点
        err.x = std::clamp(err.x, -0.3f, 0.3f);  // 限幅防止积累过大
    } else {
        err.x = 0.0f;  // 倾角过大时不做位置控制
    }
    
    // 速度误差
    err.dx     = x.dx - design.dx;  // design.dx 来自遥控器
    
    // 腿角度误差
    err.theta  = x.theta;
    err.dtheta = x.dtheta;
}

float robot::legharmony()
{
    float theta_diff = LegState_l.theta - LegState_r.theta;
    float kp_harmony = 5.0f; // 和同步增益类似，但更温和
    float kd_harmony = 1.0f;

    float harmony_torque = kp_harmony * theta_diff + kd_harmony * (legVleft.vPhi - legVright.vPhi);

    return harmony_torque;
}

//加上腿长的变化
void robot::forsafe() {
  if (fabsf(ins.Pitch) > 0.5f) { // 约 28 度
    for (auto &m : motor_)
      m.setTorque(0);
  }
}

void robot::extracted() {
  uint32_t ulNotificationValue;
  if (xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotificationValue, 0) == pdTRUE) {
    if (ulNotificationValue & 0x02) { // 收到跳跃命令
      // 收到命令先进入下蹲蓄力状态，而不是直接蹬地
      current_state = STATE_JUMP_SQUAT;
    }
    if (ulNotificationValue & 0x04) { // 检测到离地
      // 只有在"蹬地"阶段检测到离地才算真离地
      // 如果在下蹲时检测到离地(因为腿主动收缩导致力变小)，那是误判，忽略之
      if (current_state == STATE_JUMP_PUSH) {
        current_state = STATE_AIR_RETRACT;
      }
    }
    if (ulNotificationValue & 0x08) { // 检测到着地
      current_state = STATE_NORMAL;
    }
  }
}

void robot::setTorque() {
  // 关节电机使用 VMC 计算的扭矩 (已经是 N·m)
  motor_[0].setTorque(-uleft.tau4  * TORQUE_TO_CODE);  // 左腿后关节
  motor_[1].setTorque(-uleft.tau1  * TORQUE_TO_CODE);  // 左腿前关节
  motor_[3].setTorque(-uright.tau4 * TORQUE_TO_CODE);  // 右腿后关节
  motor_[4].setTorque(-uright.tau1 * TORQUE_TO_CODE);  // 右腿前关节
  
  // 驱动轮使用 LQR 计算的扭矩 (N·m → 电机控制量)
  motor_[2].setTorque(u_left.T  * TORQUE_TO_CODE);
  motor_[5].setTorque(u_right.T * TORQUE_TO_CODE);
}
