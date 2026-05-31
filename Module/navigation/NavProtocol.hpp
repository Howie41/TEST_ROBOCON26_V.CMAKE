#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

extern osThreadId_t NavControlTaskHandle;

void NavControlTask(void *argument);

namespace nav_control {

extern int16_t current_x;
extern int16_t current_y;
extern int16_t current_yaw;
extern int16_t target_x;
extern int16_t target_y;
extern int16_t target_yaw;
extern bool auto_enabled;
extern bool arrived;
extern bool target_active;
extern bool arrival_reported;
extern bool high_mode_active;  // Phase 2: 高位2006着地，只能前进/后退+锁角

}  // namespace nav_control

class NavProtocol {
 public:
  enum class CmdType {
    NONE = 0,
    TARGET,
    POSITION,
    QUERY,
    STOP,
  };

  struct NavCmd {
    CmdType type;
    int16_t param1;
    int16_t param2;
    int16_t param3;
  };

  bool processByte(uint8_t byte, NavCmd &out_cmd);

  static void buildResponse(const NavCmd &cmd, char *buf, size_t buf_size);

 private:
  void reset();

  char line_buf_[32]{};
  uint8_t index_{0};
};
