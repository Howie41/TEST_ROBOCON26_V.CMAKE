/**
 * @file control_task.h
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
#include "control_task.h"
#include "NavProtocol.hpp"
#include "chassis_task.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "tracking.h"

extern PID_t pid_x;
extern PID_t pid_y;
extern PID_t pid_yaw;

osThreadId_t ControlTaskHandle;

TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd chassis_cmd{};

TypedTopicPublisher<pub_lift_cmd> lift_data_pub("lift_cmd");
pub_lift_cmd lift_cmd{};

static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
pub_Xbox_Data control_xbox_cmd{};

static bool xbox_mode_last = false;
static bool xbox_lb_last = false;
static bool xbox_rb_last = false;
//处理底盘控制输入并发布底盘指令
void Xbox_Data_Process() {
  if (ABS(control_xbox_cmd.joyLVert - 32767) > 2000) {
    chassis_cmd.linear_x_ =
        -(int)(control_xbox_cmd.joyLVert - 32767) / 32767.0f * MAX_VELOCITY;
  } else {
    chassis_cmd.linear_x_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyLHori - 32767) > 2000) {
    chassis_cmd.linear_y_ =
        (int)(control_xbox_cmd.joyLHori - 32767) / 32767.0f * MAX_VELOCITY;
  } else {
    chassis_cmd.linear_y_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyRHori - 32767) > 2000) {
    chassis_cmd.omega_ =
        -(int)(control_xbox_cmd.joyRHori - 32767) / 32767.0f * MAX_VELOCITY;
  } else {
    chassis_cmd.omega_ = 0.0f;
  }
}
//处理升降控制输入并发布升降指令
static bool xbox_y_last = false;
static bool xbox_a_last = false;
static TickType_t xbox_y_press_tick = 0;
static TickType_t xbox_a_press_tick = 0;
constexpr TickType_t kLiftTapTimeout = pdMS_TO_TICKS(300);  // 300ms以内算"点按"
void Lift_Data_Process(){
  // === Y键：按下记录时刻，松开判断是否短按 ===
    if (control_xbox_cmd.btnY && !xbox_y_last) {
        // 上升沿：记录按下时刻
        xbox_y_press_tick = xTaskGetTickCount();
    }
    // 下降沿（松开）且持续时间 < 300ms → 短按，触发去高位
    lift_cmd.request_high = false;
    if (!control_xbox_cmd.btnY && xbox_y_last) {
        if ((xTaskGetTickCount() - xbox_y_press_tick) < kLiftTapTimeout) {
            lift_cmd.request_high = true;
        }
    }
    xbox_y_last = control_xbox_cmd.btnY;
    if (control_xbox_cmd.btnA && !xbox_a_last) {
        xbox_a_press_tick = xTaskGetTickCount();
    }
    lift_cmd.request_low = false;
    if (!control_xbox_cmd.btnA && xbox_a_last) {
        if ((xTaskGetTickCount() - xbox_a_press_tick) < kLiftTapTimeout) {
            lift_cmd.request_low = true;
        }
    }
    xbox_a_last = control_xbox_cmd.btnA;
    lift_cmd.lift_up = control_xbox_cmd.btnY;
    lift_cmd.lift_down = control_xbox_cmd.btnA;

  if(ABS(control_xbox_cmd.joyRVert - 32767) > 2000){
    lift_cmd.lift_2006_input = (int)(control_xbox_cmd.joyRVert - 32767) / 32767.0f * MAX_LIFT_VELOCITY;
  } else {
    lift_cmd.lift_2006_input = 0.0f;
  }
}
  


static bool consumeModeSwitch(bool current_state) {
  const bool rising_edge = current_state && !xbox_mode_last;
  xbox_mode_last = current_state;
  return rising_edge;
}

static bool consumeButtonRisingEdge(bool current_state, bool *last_state) {
  const bool rising_edge = current_state && !(*last_state);
  *last_state = current_state;
  return rising_edge;
}

void controlInit() {
  if (!chassis_data_pub.IsValid()) {
    return;
  }
  if (!control_xbox_sub.IsValid()) {
    return;
  }
  if (!lift_data_pub.IsValid()) {
  return;
}
}

void controlTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

  controlInit();
  for (;;) {
    if (control_xbox_sub.TryGet(&control_xbox_cmd)) {
      if (consumeModeSwitch(control_xbox_cmd.btnXbox)) {
        nav_control::auto_enabled = !nav_control::auto_enabled;

        if (nav_control::auto_enabled) {
          nav_control::target_x = nav_control::current_x;
          nav_control::target_y = nav_control::current_y;
          nav_control::target_yaw = nav_control::current_yaw;
          nav_control::arrived = false;
          nav_control::arrival_reported = false;
          PID_Init(&pid_x);
          PID_Init(&pid_y);
          PID_Init(&pid_yaw);
        }
      }

      if (!nav_control::auto_enabled) {
        if (consumeButtonRisingEdge(control_xbox_cmd.btnLB, &xbox_lb_last)) {
          chassis_action::requestYawRotateCcw90();
        }

        if (consumeButtonRisingEdge(control_xbox_cmd.btnRB, &xbox_rb_last)) {
          chassis_action::requestYawRotateCw90();
        }

        Xbox_Data_Process();
        if (chassis_action::yawRotateActive()) {
          chassis_cmd.linear_x_ = 0.0f;
          chassis_cmd.linear_y_ = 0.0f;
          chassis_cmd.omega_ = 0.0f;
        }
        Lift_Data_Process();
        lift_data_pub.Publish(lift_cmd);
        chassis_cmd.nav_mode_ = false;
        chassis_data_pub.Publish(chassis_cmd);
      }
    }

    vTaskDelayUntil(&currentTime, 5);
  }
}
