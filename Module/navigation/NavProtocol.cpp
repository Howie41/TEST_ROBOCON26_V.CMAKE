#include "NavProtocol.hpp"

#include "UsbPort.hpp"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

extern volatile float g_chassis_yaw_deg;

PID_t pid_x = {
    .Kp = 1.8f,
    .Ki = 0.13f,
    .Kd = 0.05f,
    .MaxOut = 1000.0f,
    .IntegralLimit = 100.0f,
    .DeadBand = 10.0f,
    .Improve = Integral_Limit,
};

PID_t pid_y = {
    .Kp = 1.8f,
    .Ki = 0.13f,
    .Kd = 0.05f,
    .MaxOut = 1000.0f,
    .IntegralLimit = 100.0f,
    .DeadBand = 10.0f,
    .Improve = Integral_Limit,
};

PID_t pid_yaw = {
    .Kp = 0.1f,
    .Ki = 0.001f,
    .Kd = 0.005f,
    .MaxOut = 3.0f,
    .IntegralLimit = 0.5f,
    .DeadBand = 3.0f,
    .Improve = Integral_Limit,
};

// Phase 2: 高位2006导航 — 距离→速度PID
PID_t pid_high_distance = {
    .Kp = 8.0f,
    .Ki = 0.05f,
    .Kd = 0.0f,
    .MaxOut = 500.0f,
    .IntegralLimit = 200.0f,
    .DeadBand = 5.0f,
    .Improve = Integral_Limit,
};

// Phase 2: 高位2006导航 — yaw锁角PID (输出2006 RPM)
PID_t pid_high_yaw = {
    .Kp = 15.0f,
    .Ki = 0.02f,
    .Kd = 0.0f,
    .MaxOut = 300.0f,
    .IntegralLimit = 150.0f,
    .DeadBand = 0.5f,
    .Improve = Integral_Limit,
};

static TypedTopicPublisher<pub_chassis_cmd> chassis_cmd_pub("chassis_cmd");
static TypedTopicPublisher<pub_high_nav_cmd> high_nav_pub("high_nav_cmd");

// PcCom 导航事件发布者: 全局可见，lift_task 也可使用
TypedTopicPublisher<pc_nav_event_t> pc_nav_event_pub("pc_nav_event_pub");

namespace nav_control {
int16_t current_x = 0;
int16_t current_y = 0;
int16_t current_yaw = 0;
int16_t pc_reported_yaw = 0;
int16_t target_x = 0;
int16_t target_y = 0;
int16_t target_yaw = 0;
bool auto_enabled = false;
bool arrived = false;
bool target_active = false;
bool arrival_reported = false;
bool high_mode_active = false;

// 位置更新时间戳: PcCom 收到位置上报时刷新，NavControlTask 用于超时检测
TickType_t g_last_position_update_tick = 0;

void updatePositionTimestamp() {
  g_last_position_update_tick = xTaskGetTickCount();
}

void resetAllPIDs() {
  PID_Init(&pid_x);
  PID_Init(&pid_y);
  PID_Init(&pid_yaw);
  PID_Init(&pid_high_distance);
  PID_Init(&pid_high_yaw);
}

}  // namespace nav_control

namespace {

constexpr TickType_t kPositionTimeoutTicks = pdMS_TO_TICKS(200);

bool isPositionFresh(TickType_t now) {
  return (nav_control::g_last_position_update_tick != 0U) &&
         ((now - nav_control::g_last_position_update_tick) <= kPositionTimeoutTicks);
}

float normalizeDeg(float angle_deg) {
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

void publishAutoStopCmd() {
  pub_chassis_cmd cmd{};
  cmd.nav_mode_ = true;
  chassis_cmd_pub.Publish(cmd);
}

void reportArrivalOnce() {
  if (!nav_control::target_active || nav_control::arrival_reported) {
    return;
  }

  // 先设标志，防止重复发布
  nav_control::arrival_reported = true;

  // 发布到达事件到 PcCom topic → ProcessTx 用二进制协议发送
  pc_nav_event_t evt{0x0201};
  pc_nav_event_pub.Publish(evt);

}

}  // namespace

void NavProtocol::reset() {
  index_ = 0;
  memset(line_buf_, 0, sizeof(line_buf_));
}

bool NavProtocol::processByte(uint8_t byte, NavCmd &out_cmd) {
  if (byte == '\n') {
    line_buf_[index_] = '\0';
    const char cmd_char = static_cast<char>(toupper(line_buf_[0]));
    int16_t p1 = 0;
    int16_t p2 = 0;
    int16_t p3 = 0;

    if (cmd_char == 'T') {
      const int num_read = sscanf(line_buf_, "%*c %hd %hd %hd", &p1, &p2, &p3);
      if (num_read >= 2) {
        out_cmd.type = CmdType::TARGET;
        out_cmd.param1 = p1;
        out_cmd.param2 = p2;
        out_cmd.param3 = (num_read >= 3) ? p3 : 0;
        reset();
        return true;
      }
    } else if (cmd_char == 'P') {
      const int num_read = sscanf(line_buf_, "%*c %hd %hd", &p1, &p2);
      if (num_read >= 2) {
        out_cmd.type = CmdType::POSITION;
        out_cmd.param1 = p1;
        out_cmd.param2 = p2;
        out_cmd.param3 = 0;
        reset();
        return true;
      }
    } else if (cmd_char == 'Q') {
      out_cmd.type = CmdType::QUERY;
      out_cmd.param1 = 0;
      out_cmd.param2 = 0;
      out_cmd.param3 = 0;
      reset();
      return true;
    } else if (cmd_char == 'S') {
      out_cmd.type = CmdType::STOP;
      out_cmd.param1 = 0;
      out_cmd.param2 = 0;
      out_cmd.param3 = 0;
      reset();
      return true;
    }

    reset();
    return false;
  }

  if (byte != '\r' && index_ < sizeof(line_buf_) - 1) {
    line_buf_[index_++] = static_cast<char>(byte);
  }
  return false;
}

void NavProtocol::buildResponse(const NavCmd &cmd, char *buf, size_t buf_size) {
  const TickType_t now = xTaskGetTickCount();

  switch (cmd.type) {
    case CmdType::TARGET:
      nav_control::target_x = cmd.param1;
      nav_control::target_y = cmd.param2;
      nav_control::target_yaw = cmd.param3;
      nav_control::auto_enabled = true;
      nav_control::arrived = false;
      nav_control::target_active = true;
      nav_control::arrival_reported = false;
      nav_control::resetAllPIDs();
      break;

    case CmdType::POSITION:
      nav_control::current_x = cmd.param1;
      nav_control::current_y = cmd.param2;
      nav_control::g_last_position_update_tick = now;
      snprintf(buf, buf_size, "POS OK X=%hd Y=%hd\n",
               nav_control::current_x, nav_control::current_y);
      break;

    case CmdType::QUERY:
      snprintf(buf, buf_size,
               "STATE X=%hd Y=%hd YAW=%hd AUTO=%d POS_FRESH=%d ARRIVED=%d TARGET=%d\n",
               nav_control::current_x, nav_control::current_y, nav_control::current_yaw,
               nav_control::auto_enabled ? 1 : 0,
               isPositionFresh(now) ? 1 : 0,
               nav_control::arrived ? 1 : 0,
               nav_control::target_active ? 1 : 0);
      break;

    case CmdType::STOP:
      nav_control::auto_enabled = false;
      nav_control::arrived = false;
      nav_control::target_active = false;
      nav_control::arrival_reported = false;
      snprintf(buf, buf_size, "STOP OK\n");
      break;

    default:
      snprintf(buf, buf_size, "UNKNOWN\n");
      break;
  }
}

osThreadId_t NavControlTaskHandle;

void NavControlTask(void *argument) {
  (void)argument;
  TickType_t lastWakeTime = xTaskGetTickCount();

  PID_Init(&pid_x);
  PID_Init(&pid_y);
  PID_Init(&pid_yaw);
  PID_Init(&pid_high_distance);
  PID_Init(&pid_high_yaw);

  for (;;) {
    const TickType_t now = xTaskGetTickCount();

    nav_control::current_yaw = static_cast<int16_t>(g_chassis_yaw_deg);

    if (nav_control::auto_enabled) {
      if (!isPositionFresh(now)) {
        nav_control::arrived = false;
        publishAutoStopCmd();
        vTaskDelayUntil(&lastWakeTime, 10);
        continue;
      }

      // ===== Phase 2: 高位2006导航分支 =====
      if (nav_control::high_mode_active) {
        const float error_x = static_cast<float>(nav_control::target_x - nav_control::current_x);
        const float error_y = static_cast<float>(nav_control::target_y - nav_control::current_y);
        const float dist_error = sqrtf(error_x * error_x + error_y * error_y);

        // 用现有车身坐标转换，得到目标在车体坐标系中的方向
        const float yaw_rad =
            static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;
        const float error_x_body =  error_x * cosf(yaw_rad) + error_y * sinf(yaw_rad);
        const float error_y_body = error_x * sinf(yaw_rad) - error_y * cosf(yaw_rad);

        // 车体坐标系中，atan2(error_y_body, error_x_body) = 需要转多少角度才能对准目标
        const float heading_error =
            atan2f(error_y_body, error_x_body) * 180.0f / 3.14159f;

        pub_high_nav_cmd high_cmd{};
        high_cmd.active = true;

        if (fabsf(heading_error) > 1.0f) {
          // ROTATE: 原地旋转对准目标方向
          high_cmd.forward_speed = 0.0f;
          high_cmd.omega = PID_Calculate(&pid_high_yaw, 0.0f, heading_error);
        } else {
          // DRIVE: 直走 + yaw锁角
          high_cmd.forward_speed =
              PID_Calculate(&pid_high_distance, 0.0f, dist_error);
          high_cmd.omega = PID_Calculate(&pid_high_yaw, 0.0f, heading_error);
        }

        // 限幅
        if (high_cmd.forward_speed > 500.0f) high_cmd.forward_speed = 500.0f;
        if (high_cmd.forward_speed < -500.0f) high_cmd.forward_speed = -500.0f;
        if (high_cmd.omega > 500.0f) high_cmd.omega = 500.0f;
        if (high_cmd.omega < -500.0f) high_cmd.omega = -500.0f;

        const bool reached = (dist_error < 10.0f);
        nav_control::arrived = reached;
        if (reached) {
          high_cmd.request_lower = true;
          reportArrivalOnce();
        } else {
          nav_control::arrival_reported = false;
        }

        high_nav_pub.Publish(high_cmd);
        vTaskDelayUntil(&lastWakeTime, 10);
        continue;
      }

      // ===== 低位: 全向轮导航（现有逻辑） =====
      const float error_x = static_cast<float>(nav_control::target_x - nav_control::current_x);
      const float error_y = static_cast<float>(nav_control::target_y - nav_control::current_y);
      const float error_yaw = normalizeDeg(static_cast<float>(nav_control::target_yaw - nav_control::current_yaw));

      const float yaw_rad =
          static_cast<float>(nav_control::current_yaw) * 3.14159f / 180.0f;

      // 坐标映射调整：
      // - 世界 X+ → 车体向前
      // - 世界 Y+ → 车体向左
      const float error_x_body =  error_x * cosf(yaw_rad) + error_y * sinf(yaw_rad);
      const float error_y_body = error_x * sinf(yaw_rad) - error_y * cosf(yaw_rad);

      const float vx = PID_Calculate(&pid_x, 0.0f, error_x_body);
      const float vy = PID_Calculate(&pid_y, 0.0f, error_y_body);
      const float omega = PID_Calculate(&pid_yaw, 0.0f, error_yaw);

      pub_chassis_cmd cmd{};
      cmd.linear_x_ = vx / 1000.0f;
      cmd.linear_y_ = vy / 1000.0f;
      cmd.omega_ = omega;
      cmd.nav_mode_ = true;
      chassis_cmd_pub.Publish(cmd);

      const float dist_error = sqrtf(error_x * error_x + error_y * error_y);
      const bool reached = (dist_error < 10.0f) && (fabsf(error_yaw) < 3.0f);

      nav_control::arrived = reached;
      if (reached) {
        reportArrivalOnce();
      } else {
        nav_control::arrival_reported = false;
      }
    }

    vTaskDelayUntil(&lastWakeTime, 10);
  }
}
