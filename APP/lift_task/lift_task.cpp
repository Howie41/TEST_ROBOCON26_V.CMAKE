/**
 * @file lift_task.cpp
 * @author YE
 * @brief 抬升任务实现
 * @version 0.1
 * @date 2026-05-22
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "lift_task.h"

#include "Motor.hpp"
#include "NavProtocol.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"

extern C610Motor lift_2006_motor1;
extern C610Motor lift_2006_motor2;
extern C620Motor lift_3508_motor1;
extern C620Motor lift_3508_motor2;

// 引用 NavProtocol.cpp 中定义的全局导航事件发布者
extern TypedTopicPublisher<pc_nav_event_t> pc_nav_event_pub;

osThreadId_t LiftTaskHandle;

static TypedTopicSubscriber<pub_lift_cmd> lift_cmd_sub("lift_cmd", 8);
static pub_lift_cmd lift_cmd{};

static TypedTopicSubscriber<pub_high_nav_cmd> high_nav_sub("high_nav_cmd", 4);
static pub_high_nav_cmd high_nav_cmd{};

extern volatile float g_chassis_yaw_deg;

float lift_2006_speed = 0.0f;
float lift_3508_target_pos = 0.0f;
float lift_3508_pos_pid_out = 0.0f;
bool lift_3508_hold_enable = false;
bool lift_3508_manual_last = false;

float lift_3508_motor1_pos = 0.0f;
float lift_3508_motor2_pos = 0.0f;
float lift_3508_motor1_speed = 0.0f;
float lift_3508_motor2_speed = 0.0f;

float lift_3508_avg_pos = 0.0f;
float lift_3508_diff_pos = 0.0f;

float lift_3508_base_speed = 0.0f;
float lift_3508_sync_pid_out = 0.0f;
float lift_3508_motor1_ref_speed = 0.0f;
float lift_3508_motor2_ref_speed = 0.0f;

float lift_2006_motor1_pid_out = 0.0f;
float lift_2006_motor2_pid_out = 0.0f;
float lift_3508_motor1_pid_out = 0.0f;
float lift_3508_motor2_pid_out = 0.0f;

constexpr float MAX_LIFT_2006_SPEED = 600.0f;
constexpr float MAX_LIFT_3508_SPEED = 120.0f;
constexpr float MAX_LIFT_3508_SYNC_COMP = 30.0f;

constexpr float LIFT_RISE_SPEED    = 120.0f;   // 自动上升速度 (3508 RPM)
constexpr float LIFT_FALL_SPEED    = 120.0f;   // 自动下降速度 (可以和上升不同)
constexpr float LIFT_POS_TOLERANCE =  5.0f;   // 位置到达判定容差 (度)

constexpr float LIFT_LOW_POS = 250.0f;
constexpr float LIFT_HIGH_POS = -560.0f;

constexpr float LIFT_2006_MOTOR1_DIR = 1.0f;
constexpr float LIFT_2006_MOTOR2_DIR = -1.0f;
constexpr float LIFT_3508_MOTOR1_DIR = 1.0f;
constexpr float LIFT_3508_MOTOR2_DIR = 1.0f;

enum class Lift3508Mode {
 MANUAL,
 TARGETING,
};
static Lift3508Mode lift_3508_mode = Lift3508Mode::MANUAL;
PID_t lift_2006_motor1_pid = {
    .Kp = 100.0f, .Ki = 10.0f, .Kd = 0.0f, .MaxOut = 20000, .DeadBand = 0.3f, .Improve = NONE};
PID_t lift_2006_motor2_pid = {
    .Kp = 100.0f, .Ki = 10.0f, .Kd = 0.0f, .MaxOut = 20000, .DeadBand = 0.3f, .Improve = NONE};

PID_t lift_3508_motor1_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.3f, .MaxOut = 16000, .DeadBand = 0.1f, .Improve = NONE};
PID_t lift_3508_motor2_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.3f, .MaxOut = 16000, .DeadBand = 0.1f, .Improve = NONE};
PID_t lift_3508_pos_pid = {
    .Kp = 9.0f, .Ki = 0.1f, .Kd = 0.0f, .MaxOut = MAX_LIFT_3508_SPEED, .DeadBand = 0.1f, .Improve = NONE};
PID_t lift_3508_sync_pid = {
    .Kp = 0.28f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = MAX_LIFT_3508_SYNC_COMP, .DeadBand = 0.1f, .Improve = NONE};

// Phase 2: 高位模式手动yaw锁角PID
PID_t high_yaw_lock_pid = {
    .Kp = 15.0f, .Ki = 0.02f, .Kd = 0.0f, .MaxOut = 300.0f,
    .IntegralLimit = 150.0f, .DeadBand = 0.5f, .Improve = Integral_Limit,
};
static float high_yaw_lock_ref = 0.0f;
static bool high_was_active = false;

static inline float normalizeDeg(float angle_deg) {
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

static inline void liftInit(void) {
  PID_Init(&lift_2006_motor1_pid);
  PID_Init(&lift_2006_motor2_pid);
  PID_Init(&lift_3508_motor1_pid);
  PID_Init(&lift_3508_motor2_pid);
  PID_Init(&lift_3508_pos_pid);
  PID_Init(&lift_3508_sync_pid);
  PID_Init(&high_yaw_lock_pid);

  lift_3508_motor1_pos = lift_3508_motor1.getCurrentSumPos();
  lift_3508_motor2_pos = lift_3508_motor2.getCurrentSumPos();
  lift_3508_avg_pos = (lift_3508_motor1_pos + lift_3508_motor2_pos) / 2.0f;
  lift_3508_diff_pos = lift_3508_motor1_pos - lift_3508_motor2_pos;

  //lift_3508_target_pos = LIFT_LOW_POS;
  lift_3508_target_pos = lift_3508_avg_pos;
  lift_3508_hold_enable = true;
  lift_3508_manual_last = false;

  if (!lift_cmd_sub.IsValid()) {
    return;
  }
}

static inline void Lift_Data_Process(void) {
if (lift_cmd.request_high) {
    lift_3508_target_pos = LIFT_HIGH_POS;
    lift_3508_mode = Lift3508Mode::TARGETING;
    lift_3508_hold_enable = false;
    PID_Init(&lift_3508_pos_pid);
  } else if (lift_cmd.request_low) {
    lift_3508_target_pos = LIFT_LOW_POS;
    lift_3508_mode = Lift3508Mode::TARGETING;
    lift_3508_hold_enable = false;
    PID_Init(&lift_3508_pos_pid);
  }
  lift_2006_speed = lift_cmd.lift_2006_input * MAX_LIFT_2006_SPEED;

  lift_3508_motor1_pos = lift_3508_motor1.getCurrentSumPos();
  lift_3508_motor2_pos = lift_3508_motor2.getCurrentSumPos();
  lift_3508_motor1_speed = lift_3508_motor1.getRawCurrentSpeed();
  lift_3508_motor2_speed = lift_3508_motor2.getRawCurrentSpeed();

  lift_3508_avg_pos = (lift_3508_motor1_pos + lift_3508_motor2_pos) / 2.0f;
  lift_3508_diff_pos = lift_3508_motor1_pos - lift_3508_motor2_pos;

 
  const bool lift_3508_manual_active =
  (lift_cmd.lift_up && !lift_cmd.lift_down) ||
  (lift_cmd.lift_down && !lift_cmd.lift_up);

  // ===== Phase 1: Y/A自动模式 或 手控模式 选一 =====
  if (lift_3508_mode == Lift3508Mode::TARGETING) {
    // --- Y/A自动去高/低位模式 ---
    // 用位置PID计算速度，正=上升，负=下降
    float target_speed = PID_Calculate(&lift_3508_pos_pid,
                                       lift_3508_avg_pos,
                                       lift_3508_target_pos);
    // 限幅：上升和下降可以不同速度
    if (target_speed > LIFT_RISE_SPEED)  target_speed = LIFT_RISE_SPEED;
    if (target_speed < -LIFT_FALL_SPEED) target_speed = -LIFT_FALL_SPEED;
    lift_3508_base_speed = target_speed;
    lift_3508_hold_enable = false;

    // 到达判定：位置误差在容差内 → 切回手控模式的hold逻辑
    if (fabsf(lift_3508_avg_pos - lift_3508_target_pos) <= LIFT_POS_TOLERANCE) {
      lift_3508_target_pos = lift_3508_avg_pos;  // 锁住当前位置
      lift_3508_mode = Lift3508Mode::MANUAL;
      lift_3508_hold_enable = true;
      PID_Init(&lift_3508_pos_pid);
    }

  } else if (lift_3508_manual_active) {
    // --- 手控模式：按住Y上升 / 按住A下降 ---
    if (lift_cmd.lift_up && !lift_cmd.lift_down) {
      lift_3508_base_speed = MAX_LIFT_3508_SPEED;
      lift_3508_hold_enable = false;
    } else if (lift_cmd.lift_down && !lift_cmd.lift_up) {
      lift_3508_base_speed = -MAX_LIFT_3508_SPEED;
      lift_3508_hold_enable = false;
    }

  } else {
    // --- 松手保持位置 ---
    if (lift_3508_manual_last) {
      lift_3508_target_pos = lift_3508_avg_pos;
      if (lift_3508_target_pos > LIFT_LOW_POS) {
        lift_3508_target_pos = LIFT_LOW_POS;
      }
      if (lift_3508_target_pos < LIFT_HIGH_POS) {
        lift_3508_target_pos = LIFT_HIGH_POS;
      }
      PID_Init(&lift_3508_pos_pid);
      PID_Init(&lift_3508_sync_pid);
      lift_3508_hold_enable = true;
    }
    if (lift_3508_hold_enable) {
      lift_3508_pos_pid_out = PID_Calculate(&lift_3508_pos_pid,
                                            lift_3508_avg_pos,
                                            lift_3508_target_pos);
      lift_3508_base_speed = lift_3508_pos_pid_out;
    } else {
      lift_3508_base_speed = 0.0f;
    }
  }
  lift_3508_sync_pid_out =
      PID_Calculate(&lift_3508_sync_pid, lift_3508_diff_pos, 0.0f);
//待调试
  lift_3508_motor1_ref_speed = lift_3508_base_speed + lift_3508_sync_pid_out;
  lift_3508_motor2_ref_speed = lift_3508_base_speed - lift_3508_sync_pid_out;

  lift_3508_manual_last = lift_3508_manual_active;
}

void liftTask(void *argument) {
  (void)argument;
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();

  liftInit();

  for (;;) {
    if (lift_cmd_sub.TryGet(&lift_cmd)) {
    }
    if (high_nav_sub.TryGet(&high_nav_cmd)) {
    }
    Lift_Data_Process();

    // ===== Phase 2: 2006 速度控制 — 手动/自动 共用yaw锁角 =====
    float high_forward = 0.0f;
    float high_omega = 0.0f;

    if (high_nav_cmd.active && nav_control::auto_enabled) {
      // 自动导航：上位机坐标→NavControlTask计算 speed/omega
      high_forward = high_nav_cmd.forward_speed;
      high_omega = high_nav_cmd.omega;
    } else if (nav_control::high_mode_active) {
      // 手动高位：Xbox右摇杆前进 + yaw锁角
      high_forward = lift_cmd.lift_2006_input * 500.0f;
      const float yaw_error =
          normalizeDeg(high_yaw_lock_ref - g_chassis_yaw_deg);
      high_omega = PID_Calculate(&high_yaw_lock_pid, 0.0f, yaw_error);
    }
    // 低位模式: high_forward=0, high_omega=0 → 2006不转

    const float motor1_ref = (high_forward + high_omega) * LIFT_2006_MOTOR1_DIR;
    const float motor2_ref = (high_forward - high_omega) * LIFT_2006_MOTOR2_DIR;

    lift_2006_motor1_pid_out =
        PID_Calculate(&lift_2006_motor1_pid,
                      lift_2006_motor1.getRawCurrentSpeed(),
                      motor1_ref);
    lift_2006_motor2_pid_out =
        PID_Calculate(&lift_2006_motor2_pid,
                      lift_2006_motor2.getRawCurrentSpeed(),
                      motor2_ref);

    // ===== high_mode_active 自动管理 =====
    const bool is_high = liftIsHigh();
    if (is_high && !high_was_active) {
      nav_control::high_mode_active = true;
      high_yaw_lock_ref = g_chassis_yaw_deg;
      PID_Init(&high_yaw_lock_pid);
      // 通知上位机: 进入高位模式
      pc_nav_event_t evt{static_cast<uint16_t>(0x0202)};
      pc_nav_event_pub.Publish(evt);
    } else if (!is_high && high_was_active) {
      nav_control::high_mode_active = false;
      // 通知上位机: 退出高位模式
      pc_nav_event_t evt{static_cast<uint16_t>(0x0203)};
      pc_nav_event_pub.Publish(evt);
    }
    high_was_active = is_high;

    // ===== 响应自动导航到达后降位请求 =====
    if (high_nav_cmd.request_lower && is_high) {
      high_nav_cmd.request_lower = false;  // 单次消费
      liftRequestLow();
    }

        lift_3508_motor1_pid_out =
        PID_Calculate(&lift_3508_motor1_pid,
                      lift_3508_motor1_speed,
                      lift_3508_motor1_ref_speed * LIFT_3508_MOTOR1_DIR);
    lift_3508_motor2_pid_out =
        PID_Calculate(&lift_3508_motor2_pid,
                      lift_3508_motor2_speed,
                      lift_3508_motor2_ref_speed * LIFT_3508_MOTOR2_DIR);

    lift_2006_motor1.setMotorCmd(lift_2006_motor1_pid_out);
    lift_2006_motor2.setMotorCmd(lift_2006_motor2_pid_out);
    lift_3508_motor1.setMotorCmd(lift_3508_motor1_pid_out);
    lift_3508_motor2.setMotorCmd(lift_3508_motor2_pid_out);

    vTaskDelayUntil(&currentTime, 1);
  }
}
void liftRequestHigh() {
  lift_3508_target_pos = LIFT_HIGH_POS;
  lift_3508_mode = Lift3508Mode::TARGETING;
  lift_3508_hold_enable = false;
  PID_Init(&lift_3508_pos_pid);
}

void liftRequestLow() {
  lift_3508_target_pos = LIFT_LOW_POS;
  lift_3508_mode = Lift3508Mode::TARGETING;
  lift_3508_hold_enable = false;
  PID_Init(&lift_3508_pos_pid);
}

bool liftAtTarget() {
  return (lift_3508_mode == Lift3508Mode::MANUAL);  // TARGETING结束后回到MANUAL=已到达
}

float liftCurrentPos() {
  return lift_3508_avg_pos;
}

bool liftIsHigh() {
  return (fabsf(lift_3508_avg_pos - LIFT_HIGH_POS) <= LIFT_POS_TOLERANCE);
}

bool highModeActive() {
  return nav_control::high_mode_active;
}
