/**
 ******************************************************************************
 * @file    velocity_kalman.h
 * @brief   速度卡尔曼滤波器 - 融合编码器速度和加速度计数据
 ******************************************************************************
 * @attention
 * 状态向量 x = [v, a]^T  (速度, 加速度)
 * 观测向量 z = [v_encoder, a_imu]^T
 * 
 * 状态转移方程: v(k) = v(k-1) + a(k-1)*dt
 *              a(k) = a(k-1)
 * 
 * 观测方程:     z_v = v + noise_encoder
 *              z_a = a + noise_imu
 ******************************************************************************
 */
#ifndef __VELOCITY_KALMAN_H
#define __VELOCITY_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kalman_filter.h"

// 速度滤波器状态索引
#define VEL_KF_V 0  // 速度
#define VEL_KF_A 1  // 加速度

// 滤波器结构体
typedef struct {
    KalmanFilter_t kf;
    
    float filtered_velocity;  // 滤波后的速度
    float filtered_accel;     // 滤波后的加速度
    
    float dt;                 // 采样周期
    
    // 配置参数
    float encoder_noise;      // 编码器测量噪声方差
    float imu_noise;          // IMU加速度测量噪声方差
    float process_noise_v;    // 速度过程噪声
    float process_noise_a;    // 加速度过程噪声
} VelocityKF_t;

/**
 * @brief 初始化速度卡尔曼滤波器
 * @param vkf 滤波器实例指针
 * @param dt 采样周期 (秒)
 * @param encoder_noise 编码器测量噪声方差 (越大越不信任编码器)
 * @param imu_noise IMU加速度测量噪声方差 (越大越不信任IMU)
 */
void VelocityKF_Init(VelocityKF_t *vkf, float dt, float encoder_noise, float imu_noise);

/**
 * @brief 更新卡尔曼滤波器
 * @param vkf 滤波器实例指针
 * @param encoder_velocity 编码器测得的速度 (m/s 或 rad/s)
 * @param imu_accel 加速度计测得的加速度 (m/s^2), 应该是去除重力后的运动加速度
 * @return 滤波后的速度
 */
float VelocityKF_Update(VelocityKF_t *vkf, float encoder_velocity, float imu_accel);

/**
 * @brief 获取滤波后的速度
 */
float VelocityKF_GetVelocity(VelocityKF_t *vkf);

/**
 * @brief 获取滤波后的加速度
 */
float VelocityKF_GetAccel(VelocityKF_t *vkf);

/**
 * @brief 重置滤波器状态
 */
void VelocityKF_Reset(VelocityKF_t *vkf);

#ifdef __cplusplus
}
#endif

#endif // __VELOCITY_KALMAN_H
