#ifndef LQR_H
#define LQR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // 轮子电机 (Wheel Motor) 反馈增益
    float kw_theta;     // 摆杆倾角
    float kw_gyro;      // 摆杆角速度
    float kw_x;         // 位移
    float kw_v;         // 速度
    float kw_phi;       // 机体倾角
    float kw_phi_v;     // 机体角速度

    // 髋关节 (Hip/Body) 反馈增益
    float kh_theta;
    float kh_gyro;
    float kh_x;
    float kh_v;
    float kh_phi;
    float kh_phi_v;
} LQR_Dual_K;

/**
 * @brief 计算给定腿长下的 LQR 增益矩阵 (6状态双输入)
 * @param L 当前腿长 (单位: 米)
 * @return LQR_Dual_K 包含所有12个增益系数
 */
LQR_Dual_K Get_LQR_Dual_K(float L);

#ifdef __cplusplus
}
#endif
#endif // LQR_H