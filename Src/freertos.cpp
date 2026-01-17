/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "robot.hpp"
#include "wrap_can.h"
#include <cstddef>
#include <cstdio>

/* C++ includes that need C++ linkage */
#include "middleware_can.hpp"
#include "motor.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "spi.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088Middleware.h"
#include "BMI088.h"
#include "wrap_dwt.h"
#include "iwdg.h"
#include "QuaternionEKF.h"
#include "Dbus.h"
#include "wrap_uart.h"
#include "pid.h"
#include "vmc.h"
#include "ins_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId legTaskHandle;
xTaskHandle TestHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// void INS_Task(void *pvParameters); // Removed: declaration comes from ins_task.h
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartRobotTask(void const * argument);
void StartMotorTask(void const * argument);
void StartLegTask(void const * argument);
void test(void *pvParameters);
void MotorTask(void * argument);
void setmotor(void * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  DWT_Init(168);
  BMI088_init(&hspi1, 0);
  IMU_QuaternionEKF_Init(10.0f, 0.001f, 1000000.0f, 0.9996f, 0.0f);
  DbusInit();
  CANInit();
  INS_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of robotTask */
  osThreadDef(robotTask, StartRobotTask, osPriorityNormal, 0, 512);
  robotTaskHandle = osThreadCreate(osThread(robotTask), NULL);

  /* definition and creation of motorTask */
  // osThreadDef(motorTask, StartMotorTask, osPriorityNormal, 0, 128);
  // motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* definition and creation of legTask */
  osThreadDef(legTask, StartLegTask, osPriorityNormal, 0, 128);
  legTaskHandle = osThreadCreate(osThread(legTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // 增加堆栈大小到 512 words (2KB)，防止 sprintf 和浮点运算导致栈溢出 HardFault
  xTaskCreate(test, "TestTask", 512, NULL, osPriorityNormal, &TestHandle);
  xTaskCreate(MotorTask, "MotorTask", 512, NULL, osPriorityNormal, NULL);
  xTaskCreate(setmotor, "settest", 256, NULL, osPriorityNormal, NULL);
  //这个任务就是在IMU_QuaternionEKF_Update(BMI088.Gyro[0], BMI088.Gyro[1], BMI088.Gyro[2], BMI088.Accel[0], BMI088.Accel[1], BMI088.Accel[2], dt);
  //的基础上减去了重力加速度的影响，得到纯粹的运动加速度
  xTaskCreate(INS_Task, "INSTask", 512, NULL, osPriorityNormal, NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRobotTask */
/**
* @brief Function implementing the robotTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotTask */
__weak void StartRobotTask(void const * argument)
{
  /* USER CODE BEGIN StartRobotTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRobotTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  /* USER CODE END StartMotorTask */
};

/* USER CODE BEGIN Header_StartLegTask */
/**
* @brief Function implementing the legTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLegTask */
__weak void StartLegTask(void const * argument)
{
  /* USER CODE BEGIN StartLegTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLegTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// void test(void *pvParameters)
// {
//     // char buffer[64];
//     // float time_start, time_end, time_delta;
    
//     for(;;)
//     {
//       // // 喂狗，防止复位 (IWDG 默认超时约 500ms)
//       // HAL_IWDG_Refresh(&hiwdg);

//       // // 记录开始时间
//       // time_start = DWT_GetTimeline_s();
      
//       // // 使用 DWT 延时 0.1 秒 (100ms)
//       // // 警告：如果延时超过 0.5s，看门狗会复位系统！
//       // DWT_Delay(0.3f); 
      
//       // // 记录结束时间
//       // time_end = DWT_GetTimeline_s();
      
//       // // 计算时间差
//       // time_delta = time_end - time_start;
      
//       // // 格式化打印 (为了兼容性，手动拆分整数和小数部分打印)
//       // // 理论上 time_delta 应该接近 0.100
//       // sprintf(buffer, "DWT Test: Delta = %d.%06d s\r\n", 
//       //         (int)time_delta, 
//       //         (int)((time_delta - (int)time_delta) * 1000000));
              
//       // HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      
//       // // 使用 osDelay 释放 CPU 权，让其他任务运行
//       // osDelay(1000);
//     }
// }

// void test(void *pvParameters)
// {
//   char buffer[128];
//   float dt = 0.001f;
//   float last_time = 0.0f;
//   float current_time = 0.0f;

//   // 初始化时间
//   last_time = DWT_GetTimeline_s();

//   for(;;)
//   {
//     // 喂狗
//     HAL_IWDG_Refresh(&hiwdg);

//     // 计算 dt
//     current_time = DWT_GetTimeline_s();
//     dt = current_time - last_time;
//     last_time = current_time;

//     // 防止 dt 过大或过小 (例如第一帧)
//     if(dt <= 0.0f || dt > 0.1f) dt = 0.001f;

//     BMI088_Read(&BMI088);
//     IMU_QuaternionEKF_Update(BMI088.Gyro[0], BMI088.Gyro[1], BMI088.Gyro[2], BMI088.Accel[0], BMI088.Accel[1], BMI088.Accel[2], dt);

//     float roll  = QEKF_INS.Roll;
//     float pitch = QEKF_INS.Pitch;
//     float yaw   = QEKF_INS.Yaw;          // -180 ~ 180
//     float total_yaw = QEKF_INS.YawTotalAngle; // 累计角度，包含圈数

//     // 格式化打印 Roll, Pitch, Yaw
//     // 手动拆分整数和小数部分打印，避免 %f 不支持的问题
//     sprintf(buffer, "Roll:%d.%02d Pitch:%d.%02d Yaw:%d.%02d T:%d.%02d\r\n", 
//             (int)roll, (int)((roll - (int)roll) * 100) < 0 ? -(int)((roll - (int)roll) * 100) : (int)((roll - (int)roll) * 100),
//             (int)pitch, (int)((pitch - (int)pitch) * 100) < 0 ? -(int)((pitch - (int)pitch) * 100) : (int)((pitch - (int)pitch) * 100),
//             (int)yaw, (int)((yaw - (int)yaw) * 100) < 0 ? -(int)((yaw - (int)yaw) * 100) : (int)((yaw - (int)yaw) * 100),
//             (int)total_yaw, (int)((total_yaw - (int)total_yaw) * 100) < 0 ? -(int)((total_yaw - (int)total_yaw) * 100) : (int)((total_yaw - (int)total_yaw) * 100)
//             );
    
//     HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), 10);

//     // 控制循环频率，约 1kHz
//     osDelay(1);
//   }
// }


void test(void *pvParameters)
{
  for(;;)
  {
    // 遍历打印所有电机信息
    for(const auto& m : motor_)
    { 
      //  if(m.id != 0x103 && m.id != 0x107)
      {
        // 打印 ID, 角度(rad), 速度(rad/s), 电压(V), 扭矩(Nm)
        //  log_printf("ID:0x%X A:%.2f \r\n", 
        //             m.id, m.angle/(2*3.14159f)*360.0f);
        //log_printf("l1output:%.2f l2output:%.2f\r\n",l1.output, l2.output);
        
      }
    }
    // log_printf("经过VMC计算得出的腿长l:%.4f,角度theta:%.3f",LegState_l.L,LegState_l.theta/(2*3.14159f)*360.0f);
    // log_printf("经过VMC计算得出的腿长l:%.4f,角度theta:%.3f\r\n",LegState_r.L,LegState_r.theta/(2*3.14159f)*360.0f);
    log_printf("xu书han，111222333，logloglog");
    //log_printf("Yaw:%.2f,Pitch:%.2f,Roll:%.2f\r\n",INS.Yaw*(57.295779513f),INS.Pitch*(57.295779513f),INS.Roll*(57.295779513f));
    //log_printf("状态变量x = [theta: %.3f,dtheta: %.3f,x: %.3f,dx :%.3f,phi: %.3f,dphi: %.3f]",x.theta*(57.295779513f),x.dtheta,x.x,x.dx,x.phi,x.dphi);

    // log_printf("t1:%.4f t4=%.4f,Fy_L:%.4f\r\n",uleft.tau1,uleft.tau4,f_left.F);

    // log_printf("----\r\n");
    
    osDelay(200); // 5Hz 刷新
  }
}



void MotorTask(void* argument)
{
    /* USER CODE BEGIN StartMotorTask */
    /* Infinite loop */
    for(;;)
    {
        // 喂狗
        //HAL_IWDG_Refresh(&hiwdg);
      
        motor::update();
        motor::Motorsend(motor::tx);
        //这样已经能得到左右腿的腿长和角度了
        robot::getInstance().updateState();
        // 调试：在 MotorTask 中打印第一个电机的数据，验证 update 是否生效
        // 既然开启了浮点打印支持，直接用 %f 即可
        // log_printf("MT: ID:0x%X A:%.2f\r\n", motor_[2].id, motor_[2].angle);
    
        // 控制循环频率，约 1kHz
        osDelay(1);
    }
    /* USER CODE END StartMotorTask */
}

/**
 * @brief 0.1的torque对应speed大概为9，在8.9和9.6之间震荡
 *        如果给正的t，轮子向R彪前方转，编码器值变大,重新上电之后驱动轮编码器貌似一直在-4到-5左右
 *        经测试，让轮子动起来的最低torque为0.02，这时候轮子几乎几乎动了一小小小小小小小点
 *        PID_Initialize(&l1.inner, 0.02f, 0.005, 0.0f, 0.0f, 0.2f);
 *        PID_Initialize(&l1.outer, 2.0f, 0.01, 1.5f, 0.0f, 15.0f);
 *        PID_SetErrLpfRatio(&l1.inner, 0.5f);
 *        交给AI调的参数，感觉效果还不错，不过在平衡点初扭矩有点小
 *        电机反馈的角度好像是弧度制
 * @param argument 
 */
void setmotor(void * argument)
{
  PID_Initialize(&l1.inner, 0.03f, 0.005, 0.0f, 0.0f, 0.3f);
	PID_Initialize(&l1.outer, 2.0f, 0.01, 1.5f, 0.0f, 20.0f);
	PID_SetErrLpfRatio(&l1.inner, 0.5f);

  PID_Initialize(&l2.inner, 0.03f, 0.005, 0.0f, 0.0f, 0.3f);
	PID_Initialize(&l2.outer, 2.0f, 0.01, 1.5f, 0.0f, 20.0f);
	PID_SetErrLpfRatio(&l2.inner, 0.5f);

  PID_Initialize(&test22, 2.0f, 0, 0.0f, 2.0f, 100.0f);
  PID_SetErrLpfRatio(&test22, 0.5f);  
  for(;;)
  {
 
    // PID_CascadeCalc(&l1, 0.5, motor_[2].angle, motor_[2].speed);
    // PID_CascadeCalc(&l2, -3.69, motor_[1].angle, motor_[1].speed);
    //motor_[2].setTorque(0.1f);
    PID_SingleCalc(&test22, 10.0f, motor_[2].speed);
    PID_CascadeCalc(&l1, 0.7f, motor_.at(0).angle, motor_.at(0).speed);
    PID_CascadeCalc(&l2, -0.7f, motor_.at(1).angle, motor_.at(1).speed);
    // motor_.at(0).setTorque(l1.output+0.1f);
    // motor_.at(1).setTorque(l2.output-0.f);
    //motor_.at(2).setTorque(l1.output);
    //motor_.at(2).setTorque(l1.output*0.60f);
    osDelay(10);
  }
}


/* USER CODE END Application */

#ifdef __cplusplus
}
#endif
