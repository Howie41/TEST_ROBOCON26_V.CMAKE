/**
 * @file state_machine_task.h
 * @author zhy (Howie41)
 * @brief 状态机任务头文件
 * @date 2026-05-24
 */

#pragma once

#include "cmsis_os2.h"
extern osThreadId_t StateMachineTaskHandle;

void stateMachineTask(void *argument);

// SpearheadRack - SHR 端头架
// StaffRack - SR 长杆架
enum class RobotState {
    READY,
    MOVE_TO_SHR,
};