/**
 ******************************************************************************
 * @file    velocity_kalman.c
 * @brief   速度卡尔曼滤波器实现 - 融合编码器速度和加速度计数据
 ******************************************************************************
 */

#include "velocity_kalman.h"
#include <string.h>

void VelocityKF_Init(VelocityKF_t *vkf, float dt, float encoder_noise, float imu_noise)
{
    vkf->dt = dt;
    vkf->encoder_noise = encoder_noise;
    vkf->imu_noise = imu_noise;
    vkf->filtered_velocity = 0;
    vkf->filtered_accel = 0;
    
    // 默认过程噪声
    vkf->process_noise_v = 0.01f;
    vkf->process_noise_a = 1.0f;
    
    // 初始化卡尔曼滤波器
    // xhatSize=2 (速度, 加速度)
    // uSize=0 (无控制输入)
    // zSize=2 (编码器速度, IMU加速度)
    Kalman_Filter_Init(&vkf->kf, 2, 0, 2);
    
    // 关闭自动调整，使用固定的H和R矩阵
    vkf->kf.UseAutoAdjustment = 0;
    
    // 初始化状态转移矩阵 F
    // | 1  dt |
    // | 0   1 |
    // v(k) = v(k-1) + a(k-1)*dt
    // a(k) = a(k-1)
    vkf->kf.F_data[0] = 1.0f;
    vkf->kf.F_data[1] = dt;
    vkf->kf.F_data[2] = 0.0f;
    vkf->kf.F_data[3] = 1.0f;
    
    // 初始化观测矩阵 H
    // | 1  0 |  => z_encoder = v
    // | 0  1 |  => z_imu = a
    vkf->kf.H_data[0] = 1.0f;
    vkf->kf.H_data[1] = 0.0f;
    vkf->kf.H_data[2] = 0.0f;
    vkf->kf.H_data[3] = 1.0f;
    
    // 初始化过程噪声协方差矩阵 Q
    // 离散化后的过程噪声矩阵 (简化版本)
    // | q_v    0   |
    // |  0    q_a  |
    vkf->kf.Q_data[0] = vkf->process_noise_v;
    vkf->kf.Q_data[1] = 0.0f;
    vkf->kf.Q_data[2] = 0.0f;
    vkf->kf.Q_data[3] = vkf->process_noise_a;
    
    // 初始化测量噪声协方差矩阵 R
    // | R_encoder    0      |
    // |    0      R_imu     |
    vkf->kf.R_data[0] = encoder_noise;
    vkf->kf.R_data[1] = 0.0f;
    vkf->kf.R_data[2] = 0.0f;
    vkf->kf.R_data[3] = imu_noise;
    
    // 初始化误差协方差矩阵 P (初始不确定性)
    vkf->kf.P_data[0] = 1.0f;
    vkf->kf.P_data[1] = 0.0f;
    vkf->kf.P_data[2] = 0.0f;
    vkf->kf.P_data[3] = 1.0f;
    
    // 设置状态最小方差，防止滤波器过度收敛
    vkf->kf.StateMinVariance[VEL_KF_V] = 0.001f;
    vkf->kf.StateMinVariance[VEL_KF_A] = 0.01f;
    
    // 初始化状态
    vkf->kf.xhat_data[VEL_KF_V] = 0.0f;
    vkf->kf.xhat_data[VEL_KF_A] = 0.0f;
}

float VelocityKF_Update(VelocityKF_t *vkf, float encoder_velocity, float imu_accel)
{
    // 更新测量向量
    vkf->kf.MeasuredVector[0] = encoder_velocity;  // 编码器速度
    vkf->kf.MeasuredVector[1] = imu_accel;         // IMU加速度
    
    // 执行卡尔曼滤波更新
    Kalman_Filter_Update(&vkf->kf);
    
    // 提取滤波后的状态
    vkf->filtered_velocity = vkf->kf.FilteredValue[VEL_KF_V];
    vkf->filtered_accel = vkf->kf.FilteredValue[VEL_KF_A];
    
    return vkf->filtered_velocity;
}

float VelocityKF_GetVelocity(VelocityKF_t *vkf)
{
    return vkf->filtered_velocity;
}

float VelocityKF_GetAccel(VelocityKF_t *vkf)
{
    return vkf->filtered_accel;
}

void VelocityKF_Reset(VelocityKF_t *vkf)
{
    vkf->kf.xhat_data[VEL_KF_V] = 0.0f;
    vkf->kf.xhat_data[VEL_KF_A] = 0.0f;
    vkf->filtered_velocity = 0.0f;
    vkf->filtered_accel = 0.0f;
    
    // 重置协方差矩阵
    vkf->kf.P_data[0] = 1.0f;
    vkf->kf.P_data[1] = 0.0f;
    vkf->kf.P_data[2] = 0.0f;
    vkf->kf.P_data[3] = 1.0f;
}
