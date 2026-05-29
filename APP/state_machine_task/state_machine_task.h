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

#ifdef __cplusplus
// SpearheadRack - SHR 端头架
// StaffRack - SR 长杆架
enum class RobotState: uint8_t {
    // 武馆
    begin = 0,            // 启动
    go_to_SHR,            // 前往端头架
    aim_at_weapon,        // 夹爪对准对应武器头
    catch_weapon,         // 夹爪夹取武器
    rotate_weapon_claw,   // 夹爪反转
    wait_for_cmd          // 等待R1指令
    // 梅林
};
#endif // __cplusplus