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
 tail_claw_msg=0x0001,
 tail_claw_msg_flase=0x0002,
 tail_claw_msg_success=0x0003,

   
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

    //示例包
    TypedTopicPublisher<tail_claw_msg>pc_tail_claw_pub_{"pc_tail_claw_pub"};

    TypedTopicSubscriber<tail_claw_msg>pc_tail_claw_sub_{"pc_tail_claw_sub",8};
};
