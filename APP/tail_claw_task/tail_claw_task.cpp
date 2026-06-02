#include "tail_claw_task.hpp"
#include "Motor.hpp"
#include "pid_controller.h"
#include <cstdint>

#include "control_task.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include"topic_pool.h"

constexpr float roll_reduction_ratio = 2.0f;     // 翻转的减速比
constexpr float move_max_distance = 6.0f;        // 尾部移动的最大距离,单位厘米
constexpr float move_degree_per_cm = 360.0f/(3*3.1415926f); // 齿条齿轮: 1cm平移对应电机转角

// 左右和滚的每次增量
const float move_step = 0.01f;                   // 每次移动的距离，单位厘米
const float roll_step = 0.3f;                    // 每次翻转的角度，单位度

osThreadId_t tail_claw_TaskHandle;;

extern C610Motor tail_claw_move_motor;
extern C620Motor tail_claw_roll_motor;

extern pub_Xbox_Data control_xbox_cmd;
bool weapon_claw_open = false;//武器气泵的夹紧，ture 为吸，false为放
bool KFS_claw_open = false;//KFS的夹紧，ture 为吸，false为放
bool air_pump = false;
static bool btn_share_last = false;
static bool btn_menu_last = false;
uint8_t weapon_match_state_ = 0x00;//武器在配对过程中上位机发的信号

float tail_claw_move_target_pos= 2.5f;
float tail_claw_roll_target_pos= 0.0f;
float move_cmd=0.0f;
PID_t tail_claw_move_pos_pid={
    .Kp = 0.05f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 5.0f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_move_speed_pid={
    .Kp = 700.0f,
    .Ki = 0.03f,
    .Kd = 0.02f,
    .MaxOut = 10000.0f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_roll_pos_pid={
    .Kp = 30.0f,
    .Ki = 0.0f,
    .Kd = 1.0f,
    .MaxOut = 100.0f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_roll_speed_pid={
    .Kp = 120.0f,
    .Ki = 10.0f,
    .Kd = 1.4f,
    .MaxOut = 2000.0f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

void tail_claw_init()
{ 
    PID_Init(&tail_claw_move_pos_pid);
    PID_Init(&tail_claw_move_speed_pid);
    PID_Init(&tail_claw_roll_speed_pid);
    PID_Init(&tail_claw_roll_pos_pid);
}
void weapon_open(bool open){
    if(open) weapon_claw_open=true;
    else weapon_claw_open=false;
}

void KFS_open(bool open){
    if(open) KFS_claw_open=true;
    else KFS_claw_open=false;
}

//pos为目标位置，单位为厘米，函数会返回对应的电机速度命令
float set_move_pos(float pos,PID_t *pos_pid,PID_t *speed_pid)
{
    if(pos > move_max_distance) pos = move_max_distance;
    if(pos < 0) pos = abs(pos);
    float dagree = pos*move_degree_per_cm;
    pos_pid->MaxOut = 5.0f;
    float speed_cmd=PID_Calculate(pos_pid,tail_claw_move_motor.getCurrentSumPos(),dagree);
    move_cmd=PID_Calculate(speed_pid,tail_claw_move_motor. getCurrentSpeed(),speed_cmd);
    return move_cmd;
}

//pos为目标位置，单位为度，函数会返回对应的电机速度命令
float set_roll_pos(float pos,PID_t *pos_pid,PID_t *speed_pid)
{
    float roll_pos = pos / roll_reduction_ratio;          // 根据减速比计算电机轴上的目标位置
    float speed_cmd=PID_Calculate(pos_pid,tail_claw_roll_motor.getCurrentSumPos(),roll_pos);
    return PID_Calculate(speed_pid,tail_claw_roll_motor. getCurrentSpeed(),speed_cmd);
}
static bool consumeButtonRisingEdge(bool current_state, bool *last_state) {
  const bool rising_edge = current_state && !(*last_state);
  *last_state = current_state;
  return rising_edge;
}

// PC上位机距离控制和Xbox手柄控制合并处理
// msg != nullptr: PC上位机发来了距离数据
// msg == nullptr: 仅处理Xbox手柄输入
void get_weapon_match_state(tail_claw_msg* msg)
{
    // ---- PC上位机距离控制 (仅在Xbox D-pad左右未按下时生效) ----
    if(msg != nullptr)
    {
        if(!control_xbox_cmd.btnDirLeft && !control_xbox_cmd.btnDirRight)
        {
            if(msg->distance < -5)
            {
                weapon_match_state_ = (weapon_match_state_ & ~motor_move_right) | motor_move_left;
            }
            else if(msg->distance > 5)
            {
                weapon_match_state_ = (weapon_match_state_ & ~motor_move_left) | motor_move_right;
            }
            else
            {
                weapon_match_state_ = weapon_match_state_ & ~(motor_move_left | motor_move_right);
            }
        }
    }

    // ---- Xbox D-pad 左右: 2006水平平移 ----
    if(control_xbox_cmd.btnDirLeft)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_move_right) | motor_move_left;
    }
    else if(control_xbox_cmd.btnDirRight)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_move_left) | motor_move_right;
    }else
    {
        weapon_match_state_ = weapon_match_state_ & ~(motor_move_left | motor_move_right);
    }

    // ---- Xbox D-pad 上下: 3508翻转 ----
    if(control_xbox_cmd.btnDirUp)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_roll_down) | motor_roll_up;
    }
    else if(control_xbox_cmd.btnDirDown)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_roll_up) | motor_roll_down;
    }
    else
    {
        weapon_match_state_ = weapon_match_state_ & ~(motor_roll_down | motor_roll_up);
    }

    // ---- Share: 爪子开/合 (上升沿触发) ----
    {
        static bool last_share_btn = false;
        if(control_xbox_cmd.btnShare && !last_share_btn)
        {
            weapon_claw_open = !weapon_claw_open;
        }
        last_share_btn = control_xbox_cmd.btnShare;
    }

    // ---- Menu: 气泵开关 (上升沿触发) ----
    {
        static bool last_menu_btn = false;
        if(control_xbox_cmd.btnMenu && !last_menu_btn)
        {
            air_pump = !air_pump;
        }
        last_menu_btn = control_xbox_cmd.btnMenu;
    }
}

void tail_claw_move_close()
{ 
    if(weapon_match_state_&motor_move_left)
    {
        if(tail_claw_move_target_pos > 0.0f) // 添加边界检查，确保目标位置不小于0
        {
            tail_claw_move_target_pos -= move_step;
        }else
        {
            tail_claw_move_target_pos = 0.0f;
        }
    }else if(weapon_match_state_&motor_move_right) {
        if(tail_claw_move_target_pos < move_max_distance) // 添加边界检查，确保目标位置不大于最大距离
        {
            tail_claw_move_target_pos += move_step;
        }else
        {
            tail_claw_move_target_pos = move_max_distance;
        }
    }

    
    if(weapon_match_state_&motor_roll_down)
    {
        tail_claw_roll_target_pos -= roll_step;
    }else if(weapon_match_state_&motor_roll_up) {
        tail_claw_roll_target_pos += roll_step;
    }

    if(air_pump)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
    }else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
    }

    if(weapon_claw_open)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);

    }else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
    }

    float move_cmd = set_move_pos(tail_claw_move_target_pos,
                                &tail_claw_move_pos_pid,
                                &tail_claw_move_speed_pid);
    
    float roll_cmd = set_roll_pos(tail_claw_roll_target_pos,
                                &tail_claw_roll_pos_pid,
                                &tail_claw_roll_speed_pid);
    
        /*if (fabsf( tail_claw_move_target_pos - tail_claw_move_motor.getCurrentSinglePos()) < 1.0f &&
             fabsf(tail_claw_move_motor.getCurrentSpeed()) < 0.5f) {
             tail_claw_move_motor.setMotorCmd(0.0f);
                PID_Reset(&tail_claw_move_pos_pid);
                PID_Reset(&tail_claw_move_speed_pid);
        }else{ 
            tail_claw_move_motor.setMotorCmd(move_cmd);
        }*/
        tail_claw_move_motor.setMotorCmd(move_cmd);
         tail_claw_roll_motor.setMotorCmd(roll_cmd);
}  
void tail_claw_task(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    tail_claw_init();

    static int16_t last_distance = 0;
    static bool has_last_distance = false;

    for(;;)
    {
        static TypedTopicSubscriber<tail_claw_msg> tail_claw_subscriber("pc_tail_claw_pub",8);
        tail_claw_msg msg;
        if(tail_claw_subscriber.TryGet(&msg))
        {
           last_distance = msg.distance;
           has_last_distance = true;
        }

        if(has_last_distance)
        {
            tail_claw_msg saved_msg{last_distance};
            get_weapon_match_state(&saved_msg);    // 上位机+Xbox 合并控制
        }
        else
        {
            get_weapon_match_state(nullptr);       // 仅Xbox手柄控制
        }
        if (consumeButtonRisingEdge(control_xbox_cmd.btnShare, &btn_share_last)) {
                    weapon_claw_open = !weapon_claw_open;
        }

        if (consumeButtonRisingEdge(control_xbox_cmd.btnMenu, &btn_menu_last)) {
                    air_pump = !air_pump;
        }
        tail_claw_move_close();
        weapon_match_state_=0x00;//每次执行完都清零，等待下一次指令
        vTaskDelayUntil(&currentTime, 1);
    }
}


