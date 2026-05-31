#pragma once
//此处以每个部分的看作一个对象，每个对象都有自己的状态和行为
//与我们现在的代码不一样，我们把功能对象定义为类，每个功能对象都有自己的状态和行为
//实现在driver.cpp中完成,我没有写具体实现，因为具体实现会根据具体的机器人系统进行调整，这里只是一个示例接口定义


//=============
//二D坐标和姿态
//=============
struct Pose2D {
    float x;
    float y;
    float yaw;
};

//=============
//机器人系统接口，提供底盘、夹爪、视觉和通信等功能的接口
//实际实现会根据具体的机器人系统进行调整，这里只是一个示例接口定义
//=============
class Chassis {
public:
    void setTarget(const Pose2D& pose);   //目标位置和姿态
    bool arrived() const;                 //是否到达目标位置
    void stop();                           //停止
    bool blocked();                        //是否被阻挡

private:
    Pose2D target_{0.0f, 0.0f, 0.0f};
};

class Gripper {
public:
    void open();                           //打开
    void close();                          //关闭
    void hold();                           //夹紧
    void move(float offset);              //移动夹爪，offset表示移动的距离，
                                            //正值表示向外移动，负值表示向内移动

    bool opened() const;                   //是否打开
    bool hasObject() const;                //是否有物体
};

//视觉系统接口，提供视觉相关的功能
class Vision {
public:
    void updateHead();                      //更新头部信息
                     
};

//通信系统接口，提供通信相关的功能
class Comm {
public:
    void updateFromR1();                     //更新R1信息
    void sendR2Status();                     //发送R2信息
    void cansend();                           //can发送
    void getweapon();                        //获取武器信息
};

//=============
//全局对象实例
//=============
extern Chassis chassis;
extern Gripper gripper;
extern Vision vision;
extern Comm comm;