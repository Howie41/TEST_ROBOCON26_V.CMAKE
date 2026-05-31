#include<bt_core.hpp>

#include<cstdint>
#include<cstddef>

//=============
// 黑板：用于存储行为树节点之间共享的数据
//所有的节点都可以访问和修改黑板上的数据
//=============

//这仅是一个武馆的黑板，实际要全部不会这样写死在黑板里，应该是一个通用的黑板，里面有一个map
// 或者unordered_map，key是string，value是variant或者any，可以存储任意类型的数据
struct R2Blackboard
{
    // 时间
    uint32_t now_ms = 0;

    // 比赛状态
    bool match_started = false;     //比赛开始
    bool emergency_stop = false;    //比赛紧急停止

    // R1 状态
    bool r1_has_pole = false;          //R1是否有杆（针对于规则R1要有一个杆）
    bool r1_ready_assemble = false;    //R1是否准备就绪
    bool r1_assemble_pose_ok = false;  //R1装配姿态是否正确
    bool r1_left_mc = false;           //R1是否离开武馆（针对于规则R1要离开武馆）

    // R2 状态
    bool r2_has_head = false;         //R2是否有头（针对于规则R2要有一个头）
    bool r2_at_head_rack = false;     //R2是否在头架位置（针对于规则R2要在头架位置装配）
    bool r2_at_assemble_pose = false; //R2是否在装配姿态位置
    bool r2_weapon_assembled = false; //R2武器是否已装配
    bool r2_released_head = false;    //R2是否已释放头部
    bool r2_left_mc = false;          //R2是否离开武馆（针对于规则R2要离开武馆）

    // 视觉
    bool head_visible = false;         //头部是否可见
    bool head_pose_stable = false;      //头部姿态是否稳定（连续多帧满足条件）
    float head_offset_x = 0.0f;         //头部相对于目标位置的偏移，单位是厘米，正值表示头部在目标位置的右侧，负值表示在左侧


    // 快接/夹爪
    bool connector_locked = false;      //快接是否锁定
    bool gripper_opened = false;        //夹爪是否张开
    bool gripper_has_object = false;    // 其他物体是否在夹爪中

    // 底盘状态
    bool chassis_arrived = false;       //底盘是否到达目标位置    
};

extern R2Blackboard BB;