#include "PCcom.hpp"
#include "NavProtocol.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include <codecvt>
#include <cstddef>
#include <cstdint>
#include <cstring>

void PcCom::init()
{
   manager_.set_send_function([this](const uint8_t *begin,
                                    const uint8_t *end)noexcept{
    const size_t len=static_cast<size_t>(end-begin);

    if(uart_!=nullptr){
        uart_->write(begin, len);
    }else if(usb_!=nullptr){
        usb_->WriteAsync(begin, len);
    }
    });

    manager_.set_receive_function([this](Packet packet)noexcept{
        OnPacket(std::move(packet));
    });
}

void PcCom::ProcessRx() {
    if (usb_ != nullptr) {
    UsbPort::Packet rx{};
    while (usb_->Read(rx)) {
      manager_.receive(rx.data, rx.data + rx.len);
    }
  }

  if (uart_ != nullptr) {
    UartPort::Packet rx{};
    while (uart_->Read(rx)) {
      manager_.receive(rx.data, rx.data + rx.len);
    }
  }
}


void PcCom::OnPacket(Packet packet) {
  switch(packet.code()){
    // ---- tail_claw 消息 ----
    case static_cast<uint16_t>(PcCmd::tail_claw_msg):{
      if(packet.body_size()!=sizeof(tail_claw_msg)){
        return;
      }
      tail_claw_msg msg{};
      std::memcpy(&msg,packet.body_data(),sizeof(tail_claw_msg));
      pc_tail_claw_pub_.Publish(msg);
      break;
    }

    // ---- 导航: 上位机上报当前位置 (0x0101) ----
    case static_cast<uint16_t>(PcCmd::nav_position):{
      if(packet.body_size()!=sizeof(pc_nav_position_t)){
        return;
      }
      pc_nav_position_t msg{};
      std::memcpy(&msg,packet.body_data(),sizeof(msg));
      nav_control::current_x = msg.x;
      nav_control::current_y = msg.y;
      nav_control::pc_reported_yaw = msg.yaw;
      nav_control::updatePositionTimestamp();
      break;
    }

    // ---- 导航: 上位机下发目标点 (0x0102) ----
    case static_cast<uint16_t>(PcCmd::nav_target):{
      if(packet.body_size()!=sizeof(pc_nav_target_t)){
        return;
      }
      pc_nav_target_t msg{};
      std::memcpy(&msg,packet.body_data(),sizeof(msg));
      nav_control::target_x = msg.x;
      nav_control::target_y = msg.y;
      nav_control::target_yaw = msg.yaw;
      nav_control::auto_enabled = true;
      nav_control::arrived = false;
      nav_control::target_active = true;
      nav_control::arrival_reported = false;
      nav_control::resetAllPIDs();
      break;
    }

    // ---- 上/下台阶指令 (Phase 4 TODO) ----
    case static_cast<uint16_t>(PcCmd::nav_climb_up):
    case static_cast<uint16_t>(PcCmd::nav_climb_down):
      break;

    default:
      break;
  }
}
/*
void PcCom::ProcessTx()
{
  static TypedTopicSubscriber<tail_claw_msg> tail_claw_subscriber(
      "pc_tail_claw_pub", 8);

  tail_claw_msg msg{};
  if (tail_claw_subscriber.TryGet(&msg)) {
    send(static_cast<uint16_t>(PcCmd::tail_claw_msg), msg);
  }
}
*/
void PcCom::ProcessTx()
{
  // tail_claw 发送
  tail_claw_msg claw_msg{};
  if(pc_tail_claw_sub_.TryGet(&claw_msg)){
    send(static_cast<uint16_t>(PcCmd::tail_claw_msg),claw_msg);
  }

  // 导航事件发送: 订阅 pc_nav_event_pub topic
  // 事件码即消息码，直接作为 packet code 发送
  pc_nav_event_t nav_event{};
  if(pc_nav_event_sub_.TryGet(&nav_event)){
    send(nav_event.event_code, nav_event);
  }
}

template<typename T>
bool PcCom::send(uint16_t code,const T &msg)
{
  const uint8_t *begin=reinterpret_cast<const uint8_t*>(&msg);

  Packet packet{
    code,
    begin,
    begin+sizeof(T),
    gdut::build_packet
  };

  if(!packet)
  {
    return false;
  }

  manager_.send(packet);
  return true;
}
