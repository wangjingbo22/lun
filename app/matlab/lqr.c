#include "lqr.h"
#include <math.h>

// 12个状态增益，每个有4个多项式系数 (a*L^3 + b*L^2 + c*L + d)
// 顺序: 
// 0-5: 轮子电机 [theta, d_theta, x, d_x, phi, d_phi]
// 6-11: 髋关节 [theta, d_theta, x, d_x, phi, d_phi]
// 注意：以下数据是示例占位符，请运行上面的 MATLAB 代码获取真实数值填入！
const float LQR_DUAL_COEFFS[12][4] = {
    { 1538.0577,  -629.9980,   101.9511,    11.3646}, // Wheel_Theta
    {  117.3355,   -48.0502,    10.8294,     1.3606}, // Wheel_Gyro
    {  183.4809,   -74.6997,    11.3302,     1.8847}, // Wheel_X
    { -574.3611,   201.7199,   -25.7087,     4.8407}, // Wheel_V
    {  529.0393,  -282.0176,    63.4991,    -7.1700}, // Wheel_Phi
    {   90.7113,   -41.1526,     7.8520,    -0.7892}, // Wheel_PhiV
    {  444.7340,  -308.8569,    85.1101,   -11.1437}, // Hip_Theta  
    {   69.5359,   -40.1033,     9.9134,    -1.3268}, // Hip_Gyro  
    {  136.5974,   -72.8166,    16.3954,    -1.8513}, // Hip_X  
    { 1033.4102,  -397.5074,    58.8126,    -4.1363}, // Hip_V  
    { -710.6183,   289.3109,   -43.8816,    -7.2993}, // Hip_Phi  
    { -112.3489,    43.4218,    -6.1703,    -0.4974}, // Hip_PhiV  
};

LQR_Dual_K Get_LQR_Dual_K(float L) {
    LQR_Dual_K k;
    
    // 限制输入范围，防止多项式拟合在范围外发散
    if(L < 0.04f) L = 0.04f; 
    if(L > 0.14f) L = 0.14f;
    
    float t1 = L;
    float t2 = L * L;
    float t3 = L * L * L;
    
    // 利用指针强制转换，把结构体当成 float 数组遍历
    // 结构体内存布局必须与 LQR_DUAL_COEFFS 行顺序一致
    float *p = (float*)&k;
    
    for(int i = 0; i < 12; i++) {
        p[i] = LQR_DUAL_COEFFS[i][0] * t3 + 
               LQR_DUAL_COEFFS[i][1] * t2 + 
               LQR_DUAL_COEFFS[i][2] * t1 + 
               LQR_DUAL_COEFFS[i][3];
    }
    
    return k;
}