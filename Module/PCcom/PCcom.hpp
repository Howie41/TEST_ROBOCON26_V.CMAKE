#pragma once

#include "UartPort.hpp"
#include "UsbPort.hpp"
#include"transfer_protocol.hpp"
#include"topics.hpp"
#include "verification_algorithm.hpp"
#include <cstdint>
#include "topic_pool.h"

//消息类型
enum class PcCmd : uint16_t {
  tail_claw_msg         = 0x0001,
  tail_claw_msg_flase   = 0x0002,
  tail_claw_msg_success = 0x0003,

  // ---- 上位机→下位机: 导航指令 (0x01xx) ----
  nav_position   = 0x0101,  // 上位机上报当前位置 (替代旧协议 P 命令)
  nav_target     = 0x0102,  // 上位机下发目标点   (替代旧协议 T 命令)
  nav_climb_up   = 0x0103,  // 上台阶指令 (Phase 4 待实现)
  nav_climb_down = 0x0104,  // 下台阶指令 (Phase 4 待实现)

  // ---- 下位机→上位机: 导航事件 (0x02xx) ----
  nav_arrived       = 0x0201,  // 到达目标点
  nav_high_enter    = 0x0202,  // 进入高位模式 (2006 着地)
  nav_high_exit     = 0x0203,  // 退出高位模式 (2006 离地)
  nav_climb_up_ok   = 0x0204,  // 上台阶完成 (Phase 4 待实现)
  nav_climb_down_ok = 0x0205,  // 下台阶完成 (Phase 4 待实现)
};
class PcCom {
public:
    using Packet=gdut::data_packet<gdut::crc16_algorithm> ;
    using Manager=gdut::packet_manager<gdut::crc16_algorithm>;

    explicit PcCom(UsbPort &usb):usb_(&usb){}
    explicit PcCom(UartPort &uart):uart_(&uart){}


    void init();
    void ProcessTx();
    void ProcessRx();

private:
    void OnPacket(Packet packet);

    template<typename T>
    bool send(uint16_t code,const T &msg);

private:
    Manager manager_;
    UartPort *uart_{nullptr};
    UsbPort *usb_={nullptr};

    // 示例: tail_claw 收发
    TypedTopicPublisher<tail_claw_msg> pc_tail_claw_pub_{"pc_tail_claw_pub"};
    TypedTopicSubscriber<tail_claw_msg> pc_tail_claw_sub_{"pc_tail_claw_sub", 8};

    // 导航事件订阅: lift_task / NavProtocol 发布事件 → ProcessTx 发送到上位机
    // 队列深度 4，足够缓冲连续事件
    TypedTopicSubscriber<pc_nav_event_t> pc_nav_event_sub_{"pc_nav_event_pub", 4};
};
