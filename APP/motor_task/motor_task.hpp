/**
 * @file motor_task.h
 * @author FunFer
 * @brief
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <cstdint>
#include <vector>
#include "Motor.hpp"
#include "pid_controller.h"

/** @brief 电机规划系统 
 *  @note 该系统负责对注册的电机进行位置和速度控制，使用两个级联的PID控制器实现。位置PID计算目标速度，速度PID计算最终命令输出。每个电机的PID参数可以独立配置，并且位置PID的输出会根据当前运动阶段动态调整最大速度，以实现平滑的加速和减速过程。
 *  @attention 目前仅在motorTask中以固定周期调用update()函数进行更新，后续可以根据需要调整调用方式和频率。
 *  @param motor 电机对象，必须继承自MotorBase并实现相关接口。
*/


struct MotorPlanningUnit {
    MotorBase *motor;
    PID_t pos_pid;
    PID_t speed_pid;
};

class MotorPlanningSystem {
public:

    /** @brief 注册电机到规划系统
     *  @param motor 电机对象，必须继承自MotorBase并实现相关接口。
     */
    void registerMotor(MotorBase &motor) {
        motor_planning_units_[motor_count_] = new MotorPlanningUnit{
            .motor = &motor,
            .pos_pid = {
                .Kp = 45.0f,
                .Ki = 0.0f,
                .Kd = 2.5f,
                .MaxOut = 60.0f,
                .DeadBand = 0.1f
            },
            .speed_pid = {
                .Kp = 2000.0f,
                .Ki = 0.06f,
                .Kd = 1.8f,
                .MaxOut = 12000.0f,
                .DeadBand = 0.5f
            }
        };

        PID_Init(&motor_planning_units_[motor_count_]->pos_pid);
        PID_Init(&motor_planning_units_[motor_count_]->speed_pid);
        
        motor_count_++;
    }

    /** @brief 更新电机规划
     *  @note 该函数在motorTask中周期调用，用于实时更新电机命令。
     */
    void update() {
        for (uint8_t i = 0; i < motor_count_; i++) {
            motor_planning_units_[i]->pos_pid.MaxOut = motor_planning_units_[i]->motor->updateSpeedProcess();
            motor_planning_units_[i]->motor->setMotorCmd(PID_Calculate(&motor_planning_units_[i]->speed_pid, motor_planning_units_[i]->motor->getCurrentSpeed(), PID_Calculate(&motor_planning_units_[i]->pos_pid, motor_planning_units_[i]->motor->getCurrentSumPos(), motor_planning_units_[i]->motor->tar_sum_pos_)));
        }
    }
private:
    MotorPlanningUnit *motor_planning_units_[16];
    uint8_t motor_count_ = 0;
};

void motorTask(void *argument);


// static PID_t arm3508_pos_pid{
//     .Kp = 45.0f,
//     .Ki = 0.0f,
//     .Kd = 2.5f,
//     .MaxOut = 60.0f,
//     .DeadBand = 0.1f
// };
// static PID_t arm3508_speed_pid{
//     .Kp = 2000.0f,
//     .Ki = 0.06f,
//     .Kd = 1.8f,
//     .MaxOut = 12000.0f,
//     .DeadBand = 0.5f
// };
