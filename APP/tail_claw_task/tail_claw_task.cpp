#include "tail_claw_task.hpp"
#include "Motor.hpp"
#include "pid_controller.h"
#include <cstdint>

#include "control_task.h"
#include"topic_pool.h"

constexpr float roll_reduction_ratio = 2.0f;                // 翻转的减速比
constexpr float move_max_distance = 5.0f;                  // 尾部移动的最大距离,单位厘米
constexpr float move_degree_per_cm = 360.0f/(3*3.1415926f);                   //尾部的齿轮每度转动对应的线性移动距离，单位厘米

//左右和滚的每次值率
const float move_step = 0.05f;                   //每次移动的距离，单位厘米        
const float roll_step = 2.0f;                 //每次翻转的角度，单位度

osThreadId_t tail_claw_TaskHandle;;

extern C610Motor tail_claw_move_motor;
extern C620Motor tail_claw_roll_motor;

extern pub_Xbox_Data control_xbox_cmd;
bool weapon_claw_open = false;//武器气泵的夹紧，ture 为吸，false为放
bool KFS_claw_open = false;//KFS的夹紧，ture 为吸，false为放
uint8_t weapon_match_state_ = 0x00;//武器在配对过程中上位机发的信号

float tail_claw_move_target_pos= 0.0f;
float tail_claw_roll_target_pos= 0.0f;

PID_t tail_claw_move_pos_pid={
    .Kp = 75.0f,
    .Ki = 10.0f,
    .Kd = 0.03f,
    .MaxOut = 5.0f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_move_speed_pid={
    .Kp = 75.0f,
    .Ki = 10.0f,
    .Kd = 0.03f,
    .MaxOut = 10000.0f,
    .IntegralLimit = 0.35f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_roll_pos_pid={
    .Kp = 30.0f,
    .Ki = 0.0f,
    .Kd = 3.0f,
    .MaxOut = 100.0f,
    .DeadBand = 0.3f,
    .Improve = NONE,
};

PID_t tail_claw_roll_speed_pid={
    .Kp = 2000.0f,
    .Ki = 0.0f,
    .Kd = 1.4f,
    .MaxOut = 10000.0f,
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
    if(pos < 0) pos = 0;
    float dagree = pos*move_degree_per_cm;
    pos_pid->MaxOut = 5.0f;
    float speed_cmd=PID_Calculate(pos_pid,tail_claw_move_motor.getCurrentSumPos(),dagree);
    return PID_Calculate(speed_pid,tail_claw_move_motor. getCurrentSpeed(),speed_cmd);
}

//pos为目标位置，单位为度，函数会返回对应的电机速度命令
float set_roll_pos(float pos,PID_t *pos_pid,PID_t *speed_pid)
{
    float roll_pos = pos / roll_reduction_ratio;          // 根据减速比计算电机轴上的目标位置
    pos_pid->MaxOut = 5.0f;
    float speed_cmd=PID_Calculate(pos_pid,tail_claw_roll_motor.getCurrentSumPos(),roll_pos);
    return PID_Calculate(speed_pid,tail_claw_roll_motor. getCurrentSpeed(),speed_cmd);
}

//由于没有上位机，此处先以xbox来代替 
void get_weapon_match_state()
{   
    
    //左移和右移
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
    //上下翻滚
    if(control_xbox_cmd.btnDirUp)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_roll_down) | motor_roll_up;
    }
    else if(control_xbox_cmd.btnDirDown)
    {
        weapon_match_state_ = (weapon_match_state_ & ~motor_roll_up) | motor_roll_down;
    }else
    {
        weapon_match_state_ = weapon_match_state_ & ~(motor_roll_down | motor_roll_up);
    }
}

void tail_claw_move_close()
{ 

    if(weapon_match_state_&motor_move_left)
    {
        tail_claw_move_target_pos -= move_step;
    }else if(weapon_match_state_&motor_move_right) {
        tail_claw_move_target_pos += move_step;
    }

    
    if(weapon_match_state_&motor_roll_down)
    {
        tail_claw_roll_target_pos -= move_step;
    }else if(weapon_match_state_&motor_roll_up) {
        tail_claw_roll_target_pos += move_step;
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
    for(;;)
    {
        get_weapon_match_state();
        tail_claw_move_close();
        vTaskDelayUntil(&currentTime, 2);
    }
}


