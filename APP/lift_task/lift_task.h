#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

void liftTask(void *argument);

// Phase 1: 状态机可调用的抬升接口
#ifdef __cplusplus
extern "C" {
#endif

void liftRequestHigh();     // 请求自动升到高位，非阻塞
void liftRequestLow();      // 请求自动降到低位，非阻塞
bool liftAtTarget();        // 3508是否已到达目标位置
float liftCurrentPos();     // 当前3508平均位置(度)
bool liftIsHigh();          // 当前位置是否在高位判定范围内
bool highModeActive();      // Phase 2: 2006是否着地（高位=锁角前进/后退）

#ifdef __cplusplus
}
#endif
