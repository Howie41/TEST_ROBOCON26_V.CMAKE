/**
 * @file motor_task.cpp
 * @author FunFer
 * @brief
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "stm32h7xx_hal.h"
#include "motor_task.hpp"
#include "com_config.h"
#include "Motor.hpp"
#include "pid_controller.h"
#include "topics.hpp"
#include "FreeRTOS.h"
#include "task.h"


osThreadId_t Motor_TaskHandle;

extern MotorPlanningSystem motor_planning_system;

extern C620Motor arm3508_motor;
extern C610Motor arm2006_motor;


/** @brief 电机任务函数
 *  @param argument 任务参数
 */
void motorTask(void *argument) {
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();

  for (;;) {
    motor_planning_system.update();
    vTaskDelayUntil(&currentTime, 1);

  }
}
