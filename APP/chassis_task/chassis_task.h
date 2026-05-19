/**
 * @file chassis_task.h
 * @author 大帅将军
 * @brief
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "task.h"
#include "topics.hpp"

void chassisTask(void *argument);

namespace chassis_action {

void requestYawRotateDeg(float delta_deg);
void requestYawRotateCcw90();
void requestYawRotateCw90();
bool yawRotateActive();
bool takeYawRotateFinished();
float yawRotateTargetDeg();

}  // namespace chassis_action
