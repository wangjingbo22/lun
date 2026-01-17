#pragma once
#include "ins_task.h"
#include "robot.hpp"
#include "motor.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "wrap_uart.h"

// 离地检测阈值
constexpr float flag_threshold = 5.0f;

/**
 * @brief 离地检测任务
 * @note 通过判断虚拟力矩Fy是否小于阈值来判断是否离地，通过信号量通知其他任务
 * @param argument 
 */
void gound_detect(void* argument);

/**
 * @brief 平衡检测任务
 * @note 通过检测pitch角的倾斜程度来判断是否平衡
 * @param argument 
 */
void balance_detect(void* argument);

/**
 * @brief 命令检测任务
 * @note 检测遥控器是否发出跳跃命令
 * @param argument 
 */
void cmd_detect(void* argument);

void detect_task(void* argument);