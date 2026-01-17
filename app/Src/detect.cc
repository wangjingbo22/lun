#include "detect.hpp"
#include "BMI088.h"
#include "FreeRTOS.h"
#include "robot.hpp"
#include "Dbus.h"
#include <atomic>
#include <cstdint>
#include <cmath>
#include "atomic.h"
#include "wrap_dwt.h"

extern INS_t INS;
extern Dbus_t DbusData;
extern TaskHandle_t MainHandle;

std::atomic<bool> start_flag{false};
static std::atomic<uint32_t> jump_cmd_t1_ms{0};
static constexpr uint32_t kJumpDetectWindowMs = 200;

void gound_detect(void* argument)
{
    if(!start_flag.load())
        return;
    float Fy = (f_left.F + f_right.F) / 2.0f;
    // 简单的离地/着地状态机逻辑
    // 0: 在地面(初始), 1: 已离地(等待着地)
    static int jump_stage = 0; 
    // 如果当前时间距离命令时间很近，说明是刚启动检测，强制复位状态机(保险起见，感觉加不加都行)
    if ((DWT_GetTimeline_ms() - jump_cmd_t1_ms.load()) < 20) {
        jump_stage = 0;
    }

    if (jump_stage == 0) // 等待离地
    {
        if(Fy < flag_threshold) // 检测到离地
        {
            xTaskNotify(MainHandle, 0x04, eSetBits); // 通知: 离地
            jump_stage = 1; // 切换到"空中"状态
        }
    }
    else if (jump_stage == 1) // 等待着地
    {
        if(Fy > flag_threshold) // 检测到重新着地 (可以用稍大的阈值防止抖动)
        {
            xTaskNotify(MainHandle, 0x08, eSetBits); // 通知: 着地 (新增位 0x08)
            jump_stage = 0; // 重置状态机，为下次做准备
            start_flag.store(false); 
            return;
        }
    }
    //超时保底
    if((DWT_GetTimeline_ms() - jump_cmd_t1_ms.load()) > kJumpDetectWindowMs) {
        start_flag.store(false);
        jump_stage = 0; // 重置状态机，为下次做准备
    }
}


void banlance_detect(void* argument)
{
        if(fabsf(INS.Pitch) > 20.0f) //倾斜超过20度认为不平衡
        {
            //向主任务发送通知，说明当前不平衡
            xTaskNotify(MainHandle, 
                0x01, 
                eSetBits);
        }
}

void cmd_detect(void *argument)
{
    static int8_t last_s = 0;
    if(DbusData.s[1] == 1 && last_s != 1) //检测到跳跃命令
    {
        //记录跳跃命令时间
        jump_cmd_t1_ms.store(DWT_GetTimeline_ms());
        //向主任务发送通知，说明当前收到跳跃命令
        xTaskNotify(MainHandle, 
            0x02, 
            eSetBits);
        start_flag.store(true);
    }
    last_s = DbusData.s[1];
}

void detect_task(void* argument)
{
  for(;;)
  {
    gound_detect(argument);
    banlance_detect(argument);
    cmd_detect(argument);
    osDelay(1); //1kHz检测频率
  }
}