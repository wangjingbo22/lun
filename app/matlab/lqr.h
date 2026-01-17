#ifndef LQR_H
#define LQR_H

#ifdef __cplusplus
extern "C"
{
#endif

extern const float LQR_DUAL_COEFFS[8][4];
typedef struct {
    float kw_theta, kw_gyro, kw_x, kw_v; // 轮子参数
    float kh_theta, kh_gyro, kh_x, kh_v; // 髋关节参数
} LQR_Dual_K;
/**
 * @brief 得到LQR控制器反馈矩阵K
 * 
 * @param L 
 * @return LQR_K 
 */
LQR_Dual_K Get_LQR_Dual_K(float L);

#ifdef __cplusplus
}
#endif
#endif // LQR_H