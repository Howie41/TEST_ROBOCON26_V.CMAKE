#include "PCcom.hpp"
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
    case static_cast<uint16_t>(PcCmd::tail_claw_msg):{
      if(packet.body_size()!=sizeof(tail_claw_msg))
        return;

      tail_claw_msg msg{};
      std::memcpy(&msg,packet.body_data(),sizeof(tail_claw_msg));
      pc_tail_claw_pub_.Publish(msg);
      break;
    }

    //可以有很多像上面那样的
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
  //要发送什么就订阅什么，这里以tail_claw_msg为例，大家可以参考这个写其他的
  //要发送调用send就好了
  static TypedTopicSubscriber<tail_claw_msg> tail_claw_subscriber("pc_tail_claw_sub",8);
   tail_claw_msg msg{};
  if(tail_claw_subscriber.TryGet(&msg)){
    send(static_cast<uint16_t>(PcCmd::tail_claw_msg),msg);
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
