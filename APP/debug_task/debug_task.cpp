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
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "gpio.h"

#include "task.h"

#include "logger.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "pm20s.hpp"
#include "tail_claw_task.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>


osThreadId_t Debug_TaskHandle;

static inline void debugInit(void) {

}

extern Logger logger;

void debugTask(void *argument) {
  (void)argument;
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();
  debugInit();
  
  for (;;) {
    logger.log("hello, world!\n");
    vTaskDelayUntil(&currentTime, 100);
  }
}