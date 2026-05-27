/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "state_machine_task.h"
#include <atomic>
osThreadId_t StateMachineTaskHandle;



void stateMachineTask(void *argument) {
    for (;;) {
        osDelay(1);
    }
}