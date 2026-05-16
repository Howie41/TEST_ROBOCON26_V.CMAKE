/**
 * @file debug_task.cpp
 * @author 大帅将军
 * @brief 调试任务，测试用，后续可能会删除
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "debug_task.h"
#include "Motor.hpp"
#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "gpio.h"

#include "task.h"

#include "com_config.h"
#include "pid_controller.h"
#include "pm20s.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>


osThreadId_t Debug_TaskHandle;

extern DM4310Motor arm4310_motor; 
float target_position = 0.0f;
float target_speed = 5 * 360.0f;
float target_torque = 1.0f;


static inline void debugInit(void) {
  arm4310_motor.init(10.0f, 18.0f, DM4310Motor::PosWithSpeed);
  // 明确推进模式切换序列，避免状态机停滞导致不发帧
  arm4310_motor.dmMotorDisable();
  osDelay(50);
  arm4310_motor.dmMotorEnable();
  osDelay(50);
}


void debugTask(void *argument) {
  (void)argument;
  // TickType_t currentTime;
  // currentTime = xTaskGetTickCount();
  debugInit();
    
    for (;;) {
      arm4310_motor.posWithSpeedControl(target_position, target_speed);
      osDelay(20);
    }
}