#pragma once

#include "bt_core.hpp"
#include "driver.hpp"
#include <cstdint>

// =========================
//将武馆的具体动作实现写在这个文件中，动作的接口定义在driver.hpp中，
//每个动作节点会调用driver.hpp中定义的接口来执行具体的动作，
//每个叶子节点负责执行一个具体的动作，比如移动到武器架位置、夹取头部、装配武器、等待 R1 离开武馆、离开武馆等
//每个叶子节点内部可以有一个简单的状态机来管理动作的执行

//=========================
//时间节点
//=========================
class TickTimer {
public:
    void start(uint32_t now_ms);     //开始计时，记录开始时间
    void reset();                    //重置计时器，清除开始时间和状态
    bool started() const;            //计时器是否开始  
    bool timeout(uint32_t now_ms, uint32_t duration_ms) const;       //计时器是否超时
 
private:
    uint32_t start_ms_ = 0;
    bool started_ = false;
};

// =========================
// 叶子节点
// =========================

//这是移动到头部支架的节点
class NodeMoveToHeadRack : public BTNode {
public:
    explicit NodeMoveToHeadRack(Chassis& chassis);     //主要是绑定底盘

    BT_Status tick() override;                        //不断被调用，直到返回成功或失败
    void reset() override;                             //重置状态

private:
    Chassis& chassis_;
    TickTimer timer_;                  
};

//这是移动到组装位置的节点---组装和支架是分开的，因为组装位置可能需要更精确的控制，而支架位置可能只需要粗略的移动
//后续可以合并
class NodeMoveToAssemblePose : public BTNode {
public:
    explicit NodeMoveToAssemblePose(Chassis& chassis);

    BT_Status tick() override;
    void reset() override;

private:
    Chassis& chassis_;
};



//这是移动到武馆出口的节点
class NodeMoveToMCExit : public BTNode {
public:
    explicit NodeMoveToMCExit(Chassis& chassis);

    BT_Status tick() override;
    void reset() override;

private:
    Chassis& chassis_;
};


//更新视觉的节点
class NodeUpdateVisionHead : public BTNode {
public:
    explicit NodeUpdateVisionHead(Vision& vision);

    BT_Status tick() override;

private:
    Vision& vision_;
};

//拿取武器头的节点
class NodePickHead : public BTNode {
public:
    NodePickHead( Gripper& gripper, Vision& vision);

    BT_Status tick() override;
    void reset() override;

private:
    enum class Step {        
        FineAlign,          //根据视觉信息微调位置    
        CloseGripper,       //抓取 
        RollGripper,        //翻转夹爪，防止头部掉落      
        Done                //抬起
    };

    Step step_ = Step::FineAlign;
    Gripper& gripper_;
    Vision& vision_;
};

//组装武器的节点
class NodeAssembleWeapon : public BTNode {
public:
    NodeAssembleWeapon(Chassis& chassis, Gripper& gripper, Vision& vision);

    BT_Status tick() override;
    void reset() override;

private:
    enum class Step {
        R2haveready,          //R2准备就绪，等待R1准备就绪     
        MoveToAssemblePose,   //移动到组装位置
        FineAlign,           //微调
        InsertHead,           //插入头
        CheckLocked,           //检查锁
        ReleaseHead,           //释放头
        Retreat,               //后退
        Done                  //完成
    };

    Step step_ = Step::R2haveready;

    Chassis& chassis_;
    Gripper& gripper_;
    Vision& vision_;
};


//等待R1离开武馆的节点
class NodeWaitR1LeaveMC : public BTNode {
public:
    explicit NodeWaitR1LeaveMC(Chassis& chassis);  

    BT_Status tick() override;


private:
    enum class Step {
        WaitR1LeaveMC,        //等待R1离开武馆
        Done                  //完成
    };
    Chassis& chassis_;
    Step step_ = Step::WaitR1LeaveMC;
};

