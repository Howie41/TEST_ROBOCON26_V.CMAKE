#pragma once

#include <cstdint>
#include <cstddef>

//=============
// 行为树的节点状态返回
//=============

enum class BT_Status: uint8_t
{
    SUCCESS = 0,    //节点执行成功
    FAILURE,        //节点执行失败
    RUNNING,        //节点正在执行
};

//=============
// 行为树的节点基类
//=============

class BTNode{
public:
        virtual ~BTNode() = default;
        virtual BT_Status tick() = 0;           //纯虚函数，子类必须实现,
                                                // 每次行为树执行时都会调用这个函数
        virtual void reset() {};                //虚函数，子类可以选择重写, 用于重置节点状态
};

//=============
//Sequence节点：顺序执行子节点，直到一个子节点失败或正在运行
//有记忆
//=============
//作用：从前到后执行节点
//只要有一个running，整个running
//只要有一个失败，整个失败
//只有全部成功，才成功
class Sequence : public BTNode{
public:
        Sequence(BTNode **children, std::size_t num_children): children_(children), 
                                                        num_children_(num_children), 
                                                        current_index_(0) {}
        virtual BT_Status tick() 
        {
            for(std::size_t i = current_index_; i < num_children_; ++i){
                if(children_[i] == nullptr){
                   reset(); //如果子节点为空，重置状态并返回失败
                   return BT_Status::FAILURE;
                }
                
                BT_Status status = children_[i]->tick();
                if(status == BT_Status::RUNNING){
                    current_index_ = i;                     //记录当前正在执行的子节点索引
                                                            //不reset，保持当前索引，下一次继续执行这个子节点
                    return BT_Status::RUNNING;              //如果子节点正在运行，返回正在运行
                }
                else if(status == BT_Status::FAILURE){
                    reset();                                //如果子节点失败，重置状态并返回失败
                    return BT_Status::FAILURE;
                }
                //如果上面的两种都没有发生，说明子节点成功，继续执行下一个子节点
             }

            reset(); //重置状态，准备下一次执行
            return BT_Status::SUCCESS;  //所有子节点成功，返回成功
        }

        virtual void reset() override
        {
            for(std::size_t i = 0; i < num_children_; ++i){
                children_[i]->reset(); //重置所有子节点的状态
            }
            current_index_ = 0; //重置当前正在执行的子节点索引
        }  
private:
        BTNode **children_;
        std::size_t num_children_;
        std::size_t current_index_ = 0;
};


//=============
//Selector节点：选择执行子节点，直到一个子节点成功或正在运行
//有记忆
//=============
//作用：从前到后执行节点
//只要有一个running，整个running
//只要有一个成功，整个成功
//只有全部失败，才失败
class Selector : public BTNode
{
public: 
    Selector(BTNode **children, std::size_t num_children): children_(children), 
                                                        num_children_(num_children) 
                                                       {}
    virtual BT_Status tick() 
    {
        for(std::size_t i = current_index_; i < num_children_; ++i){
            if(children_[i] == nullptr){
                reset(); //如果子节点为空，重置状态并返回失败
                return BT_Status::FAILURE;
            }

            BT_Status status = children_[i]->tick();
            if(status == BT_Status::RUNNING){
                current_index_ = i;                     //记录当前正在执行的子节点索引
                return BT_Status::RUNNING;              //如果子节点正在运行，返回正在运行
            }
            else if(status == BT_Status::SUCCESS){
                reset();                                //如果子节点成功，重置状态并返回成功
                return BT_Status::SUCCESS;
            }
        }
        reset(); //如果所有子节点都失败，重置状态并返回失败
        return BT_Status::FAILURE;
    }
    virtual void reset()
    {
        for(std::size_t i = 0; i < num_children_; ++i){
            children_[i]->reset(); //重置所有子节点的状态
        }
        current_index_ = 0; //重置当前正在执行的子节点索引
    }
private:
    BTNode **children_;
    std::size_t num_children_;
    std::size_t current_index_ = 0;
};



//=============
//Parallel节点：并行执行子节点，直到一个子节点失败或正在运行
//无记忆
//=============
//作用：并行执行节点
//只要有一个running，整个running
//只要有一个失败，整个失败
//只有全部成功，才成功
class Parallel : public BTNode
{
public: 
    Parallel(BTNode **children, std::size_t num_children): children_(children), 
                                                        num_children_(num_children) 
                                                         {} 
    virtual BT_Status tick()
    {
        bool all_success = true;                      //标记是否所有子节点都成功
        
        for(std::size_t i = 0; i < num_children_; ++i){
            if(children_[i] == nullptr){
                reset(); //如果子节点为空，重置状态并返回失败
                return BT_Status::FAILURE;
            }

            if(child_done_[i]){ //如果子节点已经完成，跳过
                continue;
            }
            BT_Status status = children_[i]->tick();
            if(status == BT_Status::FAILURE){
                reset();                               //如果子节点失败，重置状态并返回失败
                return BT_Status::FAILURE;
            }
            if(status == BT_Status::SUCCESS){
                child_done_[i] = true;                  //标记子节点完成
            }
            if(status == BT_Status::RUNNING){
                all_success = false;                    //如果子节点正在运行，标记为false
                return BT_Status::RUNNING;
            }
        }

        if(all_success){

            reset();                                 //如果所有子节点都成功，重置状态准备下一次执行
            return BT_Status::SUCCESS;               //如果所有子节点都成功，返回成功
        }
    }

    virtual void reset()
    {
        for(std::size_t i = 0; i < num_children_; ++i){
            children_[i]->reset();                   //重置所有子节点的状态
            child_done_[i] = false;                    //重置子节点完成标记
        }
    }

private:
    BTNode **children_;
    std::size_t num_children_;
    bool child_done_[10] = {false};             //假设最多10个子节点，记录每个子节点是否完成
};

