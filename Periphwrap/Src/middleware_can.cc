#include "middleware_can.hpp"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "wrap_uart.h"

// 定义静态成员变量
MidCan MidCan::midcan;


void MidCan::decode(void)
{
    float angle,speed;
    angle = (*(int32_t *)&rxbuff[0] / 1000.0f );
    speed = (*(int16_t *)&rxbuff[4] / 10 * 2 * PI / 60);
    //rxheader = MidCan::getcan().rxheader;
    log_printf("angle:%f,speed:%f",
                angle,speed);
} 





