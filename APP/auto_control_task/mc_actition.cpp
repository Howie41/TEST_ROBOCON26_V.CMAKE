#include "mc_mission.hpp"
#include "blackboard.hpp"
//这是整个R2武馆任务的行为树的根节点，负责管理整个任务的流程
//它包含了几个子节点，分别负责不同的步骤，比如移动到头部支架位置、夹取头部、移动到组装位置、装配武器、等待 R1 离开武馆、离开武馆等
//这个文件主要是写具体的动作实现，会调用driver.hpp中定义的接口，
//这样可以将行为树的逻辑和具体的机器人系统实现分离，
//我就写一些示例节点，其他的节点可以按照这个模式来实现，具体的实现会根据实际情况进行调整

// =========================
// 坐标参数：需要按场地标定修改
// =========================
//以下坐标都是相对于武馆内某个固定点的坐标，单位是米，
// 头部支架位置
static constexpr Pose2D HEAD_RACK_POSE {
    0.80f,
    0.20f,
    0.0f
};

//组装位置
static constexpr Pose2D ASSEMBLE_POSE {
    1.20f,
    0.50f,
    0.0f
};

//武馆出口位置
static constexpr Pose2D MC_EXIT_POSE {
    2.00f,
    0.30f,
    0.0f
};

// =========================
// 时间节点
// =========================
void TickTimer::start(uint32_t now_ms)
{
    start_ms_ = now_ms;
    started_ = true;
}

void TickTimer::reset()
{
    start_ms_ = 0;
    started_ = false;
}

bool TickTimer::started() const
{
    return started_;
}

bool TickTimer::timeout(uint32_t now_ms, uint32_t duration_ms) const
{
    if (!started_) {
        return false;
    }

    return static_cast<uint32_t>(now_ms - start_ms_) >= duration_ms;
}
//=========================
// 示例创建叶子节点
//=========================
//这是移动到头部支架的节点
NodeMoveToHeadRack::NodeMoveToHeadRack(Chassis& chassis)
    : chassis_(chassis){}

BT_Status NodeMoveToHeadRack::tick()
{
    chassis_.setTarget(HEAD_RACK_POSE);

    if (chassis_.arrived()) {
        BB.r2_at_head_rack = true;
        return BT_Status::SUCCESS;
    }

    if ( chassis_.blocked()||timer_.timeout(BB.now_ms, 3000)) {
        return BT_Status::FAILURE;     //如果底盘被阻挡或者超时，返回失败
    }

    return BT_Status::RUNNING;  //既不失败又不成功，继续运行
}


