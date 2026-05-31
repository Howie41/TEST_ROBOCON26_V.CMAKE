#include "mc_mission.hpp"

#include "mc_actitions.hpp"
#include "mc_mission.hpp"
#include "blackboard.hpp"

//==========================
//得到头部
//==========================

//这里有个地方要注意，在C++中，类成员变量的初始化顺序是按照它们在类声明中的顺序进行初始化的。
//所以在GET_HEAD的声明中，成员变量的初始化顺序应该按照它们在实际声明需要中的顺序来写，
GET_HEAD::GET_HEAD(Chassis& chassis,Gripper& gripper,Vision& vision)
    : move_to_head_rack_node_(chassis),
      pick_head_node_(gripper, vision),
      update_vision_head_node_(vision),
      prepare_nodes_{
          &move_to_head_rack_node_,
          &update_vision_head_node_
      },
     prepare_parallel_node_(prepare_nodes_.data(), prepare_nodes_.size()),
    root_nodes_{
          &prepare_parallel_node_,
          &pick_head_node_
      },
      root_sequence_node_(root_nodes_.data(), root_nodes_.size()){}


BT_Status GET_HEAD::tick()
{
    return root_sequence_node_.tick();
}

void GET_HEAD::reset()
{
    root_sequence_node_.reset();
}

//==========================
//组装武器
//==========================
BTAssembleWeapon::BTAssembleWeapon(Chassis& chassis, Gripper& gripper, Vision& vision)
    : move_to_assemble_pose_node_(chassis),
      assemble_weapon_node_(chassis, gripper, vision),
      sequence_nodes_{
          &move_to_assemble_pose_node_,
          &assemble_weapon_node_
      },
      sequence_node_(sequence_nodes_.data(), sequence_nodes_.size()){}

BT_Status BTAssembleWeapon::tick()
{
    return sequence_node_.tick();
}

void BTAssembleWeapon::reset()
{
    sequence_node_.reset();
}

//==========================
//离开武馆
//==========================
BTExitMC::BTExitMC(Chassis& chassis)
    : wait_r1_leave_mc_node_(chassis),
      move_to_mc_exit_node_(chassis),
      sequence_nodes_{
          &wait_r1_leave_mc_node_,
          &move_to_mc_exit_node_
      },
      sequence_node_(sequence_nodes_.data(), sequence_nodes_.size()){}

BT_Status BTExitMC::tick()
{
    return sequence_node_.tick();
}

void BTExitMC::reset()
{
    sequence_node_.reset();
}


//==========================
//整个R2武馆任务的根节点
//==========================
R2MCMission::R2MCMission(Chassis& chassis, Gripper& gripper, Vision& vision, Comm& comm)
    : get_head_node_(chassis, gripper, vision),
      assemble_weapon_node_(chassis, gripper, vision),
      exit_mc_node_(chassis),
      chassis_(chassis),
      gripper_(gripper),
      vision_(vision),
      comm_(comm){}

void R2MCMission::tick()
{
    updateBlackboard();

    if (BB.emergency_stop) {
        enterError();
        return;
    }

    switch (state_) {
    case R2MCState::IDLE:
        chassis_.stop();

        if (BB.match_started) {
            state_ = R2MCState::PREPARE;
        }
        break;

    case R2MCState::PREPARE:
        state_ = R2MCState::MOVE_TO_HEAD_RACK;
        break;

    case R2MCState::MOVE_TO_HEAD_RACK: {
        BT_Status status = get_head_node_.tick();

        if (status == BT_Status::SUCCESS) {
            get_head_node_.reset();
            state_ = R2MCState::MOVE_TO_ASSEMBLE;
        } else if (status == BT_Status::FAILURE) {
            get_head_node_.reset();
            enterError();
        }
        break;
    }

    case R2MCState::GRIP_HEAD:
        state_ = R2MCState::MOVE_TO_ASSEMBLE;
        break;

    case R2MCState::MOVE_TO_ASSEMBLE: {
        BT_Status status = assemble_weapon_node_.tick();

        if (status == BT_Status::SUCCESS) {
            assemble_weapon_node_.reset();
            state_ = R2MCState::WAIT_R1_LEAVE;
        } else if (status == BT_Status::FAILURE) {
            assemble_weapon_node_.reset();
            enterError();
        }
        break;
    }

    case R2MCState::ASSEMBLE_WEAPON:
        state_ = R2MCState::WAIT_R1_LEAVE;
        break;

    case R2MCState::WAIT_R1_LEAVE:
        chassis_.stop();

        if (BB.r1_left_mc) {
            state_ = R2MCState::EXIT_MC;
        }
        break;

    case R2MCState::EXIT_MC: {
        BT_Status status = exit_mc_node_.tick();

        if (status == BT_Status::SUCCESS) {
            exit_mc_node_.reset();
            state_ = R2MCState::Done;
        } else if (status == BT_Status::FAILURE) {
            exit_mc_node_.reset();
            enterError();
        }
        break;
    }

    case R2MCState::Done:
        chassis_.stop();
        gripper_.hold();
        break;

    case R2MCState::Error:
        chassis_.stop();
        gripper_.hold();
        break;
    }
}

bool R2MCMission::isdone() const
{
    return state_ == R2MCState::Done;
}

bool R2MCMission::isfailed() const
{
    return state_ == R2MCState::Error;
}

R2MCState R2MCMission::getState() const
{
    return state_;
}

void R2MCMission::updateBlackboard()
{
    //可以在这个接口中添加更多的更新逻辑，比如从其他传感器获取信息，或者根据当前状态计算一些衍生信息等
    comm_.updateFromR1();
    vision_.updateHead();

    BB.gripper_opened = gripper_.opened();
    BB.gripper_has_object = gripper_.hasObject();
    BB.chassis_arrived = chassis_.arrived();

    comm_.sendR2Status();
}

void R2MCMission::enterError()
{
    chassis_.stop();
    gripper_.hold();

    get_head_node_.reset();
    assemble_weapon_node_.reset();
    exit_mc_node_.reset();

    state_ = R2MCState::Error;
}