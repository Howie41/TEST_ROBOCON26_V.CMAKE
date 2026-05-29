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
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"

extern C610Motor lift_2006_motor1;
extern C610Motor lift_2006_motor2;
extern C620Motor lift_3508_motor1;
extern C620Motor lift_3508_motor2;

osThreadId_t LiftTaskHandle;

static TypedTopicSubscriber<pub_lift_cmd> lift_cmd_sub("lift_cmd", 8);
static pub_lift_cmd lift_cmd{};

float lift_2006_speed = 0.0f;
float lift_3508_speed = 0.0f;
float lift_3508_target_pos = 0.0f;
float lift_3508_current_pos = 0.0f;
float lift_3508_pos_pid_out = 0.0f;
bool lift_3508_hold_enable = false;
bool lift_3508_manual_last = false;

float lift_2006_motor1_pid_out = 0.0f;
float lift_2006_motor2_pid_out = 0.0f;
float lift_3508_motor1_pid_out = 0.0f;
float lift_3508_motor2_pid_out = 0.0f;

constexpr float MAX_LIFT_2006_SPEED = 600.0f;
constexpr float MAX_LIFT_3508_SPEED = 120.0f;

constexpr float LIFT_LOW_POS = 50.0f;
constexpr float LIFT_HIGH_POS = -560.0f;

constexpr float LIFT_2006_MOTOR1_DIR = 1.0f;
constexpr float LIFT_2006_MOTOR2_DIR = -1.0f;
constexpr float LIFT_3508_MOTOR1_DIR = 1.0f;
constexpr float LIFT_3508_MOTOR2_DIR = 1.0f;

PID_t lift_2006_motor1_pid = {
    .Kp = 100.0f, .Ki = 10.0f, .Kd = 0.0f, .MaxOut = 20000, .DeadBand = 0.3f, .Improve = NONE};
PID_t lift_2006_motor2_pid = {
    .Kp = 100.0f, .Ki = 10.0f, .Kd = 0.0f, .MaxOut = 20000, .DeadBand = 0.3f, .Improve = NONE};

PID_t lift_3508_motor1_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.3f, .MaxOut = 16000, .DeadBand = 0.1f, .Improve = NONE};
PID_t lift_3508_motor2_pid = {
    .Kp = 100.0f, .Ki = 30.0f, .Kd = 0.3f, .MaxOut = 16000, .DeadBand = 0.1f, .Improve = NONE};
PID_t lift_3508_pos_pid = {
    .Kp = 8.0f, .Ki = 0.1f, .Kd = 0.0f, .MaxOut = MAX_LIFT_3508_SPEED, .DeadBand = 0.1f, .Improve = NONE};

static inline void liftInit(void) {
  PID_Init(&lift_2006_motor1_pid);
  PID_Init(&lift_2006_motor2_pid);
  PID_Init(&lift_3508_motor1_pid);
  PID_Init(&lift_3508_motor2_pid);
  PID_Init(&lift_3508_pos_pid);

  lift_3508_current_pos =
      (lift_3508_motor1.getCurrentSumPos() + lift_3508_motor2.getCurrentSumPos()) / 2.0f;
  lift_3508_target_pos = LIFT_LOW_POS;
  lift_3508_hold_enable = true;
  lift_3508_manual_last = false;

  if (!lift_cmd_sub.IsValid()) {
    return;
  }
}

static inline void Lift_Data_Process(void) {
  lift_2006_speed = lift_cmd.lift_2006_input * MAX_LIFT_2006_SPEED;
  lift_3508_current_pos =
      (lift_3508_motor1.getCurrentSumPos() + lift_3508_motor2.getCurrentSumPos()) / 2.0f;
  const bool lift_3508_manual_active =
      (lift_cmd.lift_up && !lift_cmd.lift_down) ||
      (lift_cmd.lift_down && !lift_cmd.lift_up);

  if (lift_cmd.lift_up && !lift_cmd.lift_down) {
    lift_3508_speed = MAX_LIFT_3508_SPEED;
    lift_3508_hold_enable = false;
  } else if (lift_cmd.lift_down && !lift_cmd.lift_up) {
    lift_3508_speed = -MAX_LIFT_3508_SPEED;
    lift_3508_hold_enable = false;
  } else {
    if (lift_3508_manual_last) {
      lift_3508_target_pos = lift_3508_current_pos;
      if (lift_3508_target_pos > LIFT_LOW_POS) {
        lift_3508_target_pos = LIFT_LOW_POS;
      }
      if (lift_3508_target_pos < LIFT_HIGH_POS) {
        lift_3508_target_pos = LIFT_HIGH_POS;
      }
      PID_Init(&lift_3508_pos_pid);
      lift_3508_hold_enable = true;
    }

    if (lift_3508_hold_enable) {
      lift_3508_pos_pid_out = PID_Calculate(&lift_3508_pos_pid,
                                            lift_3508_current_pos,
                                            lift_3508_target_pos);
      lift_3508_speed = lift_3508_pos_pid_out;
    } else {
      lift_3508_speed = 0.0f;
    }
  }

  lift_3508_manual_last = lift_3508_manual_active;
}

void liftTask(void *argument) {
  (void)argument;
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();

  liftInit();

  for (;;) {
    if (lift_cmd_sub.TryGet(&lift_cmd)) {
      Lift_Data_Process();
    }

    lift_2006_motor1_pid_out =
        PID_Calculate(&lift_2006_motor1_pid,
                      lift_2006_motor1.getRawCurrentSpeed(),
                      lift_2006_speed * LIFT_2006_MOTOR1_DIR);
    lift_2006_motor2_pid_out =
        PID_Calculate(&lift_2006_motor2_pid,
                      lift_2006_motor2.getRawCurrentSpeed(),
                      lift_2006_speed * LIFT_2006_MOTOR2_DIR);

    lift_3508_motor1_pid_out =
        PID_Calculate(&lift_3508_motor1_pid,
                      lift_3508_motor1.getRawCurrentSpeed(),
                      lift_3508_speed * LIFT_3508_MOTOR1_DIR);
    lift_3508_motor2_pid_out =
        PID_Calculate(&lift_3508_motor2_pid,
                      lift_3508_motor2.getRawCurrentSpeed(),
                      lift_3508_speed * LIFT_3508_MOTOR2_DIR);

    lift_2006_motor1.setMotorCmd(lift_2006_motor1_pid_out);
    lift_2006_motor2.setMotorCmd(lift_2006_motor2_pid_out);
    lift_3508_motor1.setMotorCmd(lift_3508_motor1_pid_out);
    lift_3508_motor2.setMotorCmd(lift_3508_motor2_pid_out);

    vTaskDelayUntil(&currentTime, 1);
  }
}
