#pragma once

#include<array>

#include"bt_core.hpp"
#include"mc_actitions.hpp"
#include"driver.hpp"
#include "stm32h7xx_hal.h"

enum class R2MCState {
    IDLE = 0,           //空闲状态，等待比赛开始
    PREPARE,            //准备状态，进行一些准备工作，比如初始化传感器、校准等
    MOVE_TO_HEAD_RACK,  //移动到头部支架位置
    GRIP_HEAD,          //夹取头部
    MOVE_TO_ASSEMBLE,   //移动到组装位置
    ASSEMBLE_WEAPON,    //装配武器
    WAIT_R1_LEAVE,      //等待 R1 离开武馆
    EXIT_MC,             //离开武馆
    Error,               //错误状态，进入这个状态后需要人工干预
    Done                //完成状态，任务完成后进入这个状态
};

class GET_HEAD : public BTNode {
public:
    GET_HEAD(Chassis& chassis, Gripper& gripper, Vision& vision);
     BT_Status tick() override;
     void reset() override;
private:
    NodeMoveToHeadRack move_to_head_rack_node_;
    NodeUpdateVisionHead update_vision_head_node_;
    NodePickHead pick_head_node_;

    //节点的状态
    std::array<BTNode*, 2>prepare_nodes_;
    Parallel prepare_parallel_node_;
    std::array<BTNode*, 2>root_nodes_;
    Sequence root_sequence_node_;
};  

class BTAssembleWeapon : public BTNode {
public:
    BTAssembleWeapon(Chassis& chassis, Gripper& gripper, Vision& vision);
     BT_Status tick() override;

     void reset() override;
private:
    NodeMoveToAssemblePose move_to_assemble_pose_node_;
    NodeAssembleWeapon assemble_weapon_node_;

    //节点的状态
    std::array<BTNode*, 2>sequence_nodes_;
    Sequence sequence_node_;
};  


class BTExitMC : public BTNode {
public:    
    BTExitMC(Chassis& chassis);

     BT_Status tick() override;

     void reset() override;
private:
    NodeWaitR1LeaveMC wait_r1_leave_mc_node_;
    NodeMoveToMCExit move_to_mc_exit_node_; 
    //节点的状态
    std::array<BTNode*, 2>sequence_nodes_;
    Sequence sequence_node_;
};

class R2MCMission {
public:
    R2MCMission(Chassis& chassis, Gripper& gripper, Vision& vision, Comm& comm);

    void tick();
    bool isdone() const;
    bool isfailed() const;
    R2MCState getState() const;
private:
    GET_HEAD get_head_node_;
    BTAssembleWeapon assemble_weapon_node_;
    BTExitMC exit_mc_node_;

    //更新黑板上的信息，主要是从视觉系统和通信系统获取最新的信息，更新到黑板上，供行为树的节点使用
    void updateBlackboard();
    void enterError();

    //各个部分的机构资源
    Chassis& chassis_;
    Gripper& gripper_;
    Vision& vision_;
    Comm& comm_;

    //行为树的状态
    R2MCState state_ = R2MCState::IDLE;
};   