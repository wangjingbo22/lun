#include <math.h>
#include "lqr.h"

// Dual-Input LQR Gains
// Row 0..3: K_wheel (For Wheel Motor)
// Row 4..7: K_hip   (For 5-Bar Motors)
const float LQR_DUAL_COEFFS[8][4] = {
    {-124.0217, 41.7763, -12.1902, -3.2888}, // Input1-State0
    {-24.5994, 4.7279, -1.0419, -0.4691}, // Input1-State1
    {-23.1247, 11.3810, -4.0631, 0.4988}, // Input1-State2
    {-29.7322, 14.1651, -6.5714, 0.8364}, // Input1-State3
    {-131.1640, 50.2486, -4.4359, 1.9272}, // Input2-State0
    {-32.0245, 14.8417, -1.5373, 0.2775}, // Input2-State1
    {23.1615, -11.0669, 1.9549, 0.5084}, // Input2-State2
    {33.5239, -17.8810, 3.9023, 0.8514}, // Input2-State3
};





LQR_Dual_K Get_LQR_Dual_K(float L) {
    LQR_Dual_K k;
    if(L < 0.04f) L = 0.04f; if(L > 0.14f) L = 0.14f;
    float t1 = L, t2 = L*L, t3 = L*L*L;
    float *p = (float*)&k;
    for(int i=0; i<8; i++) {
        p[i] = LQR_DUAL_COEFFS[i][0]*t3 + LQR_DUAL_COEFFS[i][1]*t2 + LQR_DUAL_COEFFS[i][2]*t1 + LQR_DUAL_COEFFS[i][3];
    }
    return k;
}