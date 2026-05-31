/**
 * @file com_config.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 全局通信配置，包含can设备、串口设备、协议解析等
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "com_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_uart.h"
#include "task.h"

#include "memory_map.h"

#include "Canbus.hpp"
#include "Hwt101.hpp"
#include "infrared_com.hpp"
#include "Motor.hpp"
#include "ROSCom.hpp"
#include "UartPort.hpp"
#include "UsbPort.hpp"
#include "XboxRemote.hpp"
#include "NavProtocol.hpp"
#include "logger.hpp"
#include "topics.hpp"
#include "topic_pool.h"
#include "usart.h"
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstring>

osThreadId_t CAN1_Send_TaskHandle;
osThreadId_t CAN2_Send_TaskHandle;
osThreadId_t CAN3_Send_TaskHandle;
osThreadId_t uart2ProcessTaskHandle;
osThreadId_t uart3ProcessTaskHandle;
osThreadId_t usbcdcProcessTaskHandle;
osThreadId_t PcComTaskHandle;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

CanBus fdcan1_bus(hfdcan1);
CanBus fdcan2_bus(hfdcan2);
CanBus fdcan3_bus(hfdcan3);

// can设备

// 底盘电机
C620Motor chassis_motor1(&fdcan3_bus, 0x201, 0, 0x200, 0);
C620Motor chassis_motor2(&fdcan3_bus, 0x202, 0, 0x200, 0);
C620Motor chassis_motor3(&fdcan3_bus, 0x203, 0, 0x200, 0);
C620Motor chassis_motor4(&fdcan3_bus, 0x204, 0, 0x200, 0);

//
C610Motor arm2006_motor(&fdcan2_bus, 0x205, 0, 0x1FF, 0);
C620Motor arm3508_motor(&fdcan2_bus, 0x206, 0, 0x1FF, 0);
//DM43xxMotor arm4310_motor(&fdcan2_bus, 0x301, 0, 0x01, 0,DM43xxMotor::PosWithSpeed);

//尾部的电机
C610Motor tail_claw_move_motor(&fdcan2_bus, 0x201, 0, 0x200, 0);
C620Motor tail_claw_roll_motor(&fdcan2_bus, 0x202, 0, 0x200, 0);


//抬升电机
C610Motor lift_2006_motor1(&fdcan1_bus, 0x201, 0, 0x200, 0);
C610Motor lift_2006_motor2(&fdcan1_bus, 0x202, 0, 0x200, 0);
C620Motor lift_3508_motor1(&fdcan1_bus, 0x203, 0, 0x200, 0);
C620Motor lift_3508_motor2(&fdcan1_bus, 0x204, 0, 0x200, 0);

// 串口外设（回调+信号量唤醒处理线程进行解包）
void onUart3RxCb(const uint8_t *data, size_t len, void *user);
void onUart2RxCb(const uint8_t *data, size_t len, void *user);
void onUart6RxCb(const uint8_t *data, size_t len, void *user);

void onUsbRxCb(const uint8_t *data, size_t len, void *user);

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[64];
UartPort uart3_port(&huart3, uart3_rx_dma, sizeof(uart3_rx_dma), uart3_tx_dma,
                    sizeof(uart3_tx_dma), onUart3RxCb, nullptr);
osSemaphoreId_t uart3_rx_semphore = NULL;

DMA_BUFFER_ATTR static uint8_t uart2_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart2_tx_dma[64];
UartPort uart2_port(&huart2, uart2_rx_dma, sizeof(uart2_rx_dma), uart2_tx_dma,
                    sizeof(uart2_tx_dma), onUart2RxCb, nullptr);
osSemaphoreId_t uart2_rx_semphore = NULL;

 // USART6 红外模块
DMA_BUFFER_ATTR static uint8_t uart6_rx_dma[UartPort::kPacketPayloadSize] = {0};
DMA_BUFFER_ATTR static uint8_t uart6_tx_dma[64] = {0};
UartPort uart6_port(&huart6, uart6_rx_dma, sizeof(uart6_rx_dma),
                            uart6_tx_dma, sizeof(uart6_tx_dma), onUart6RxCb, nullptr);

// USART10 日志
DMA_BUFFER_ATTR static uint8_t uart10_rx_dma[64] = {0};
DMA_BUFFER_ATTR static uint8_t uart10_tx_dma[Logger::BUFFER_LENGTH] = {0};
UartPort uart10_port(&huart10, uart10_rx_dma, sizeof(uart10_rx_dma),
                             uart10_tx_dma, sizeof(uart10_tx_dma), nullptr, nullptr);

// Xbox控制器（基于uart3）
XboxRemote xbox_remote(uart3_port);
TypedTopicPublisher<pub_Xbox_Data> xbox_data_pub("xbox");
pub_Xbox_Data xbox_msg;
// HWT101 陀螺仪
volatile float g_hwt101_yaw_deg = 0.0f;
//volatile float g_hwt101_pitch_deg = 0.0f;
//volatile float g_hwt101_roll_deg = 0.0f;
//同样 需要roll和pitch再开启
volatile uint32_t g_hwt101_frame_count = 0;
Hwt101Parser hwt101_parser;

// 导航协议解析器
NavProtocol nav_protocol;
// 红外通信
InfraredModule infrared_module(uart6_port);
// 日志
Logger logger(uart10_port);

// usb
osSemaphoreId_t usbcdc_rx_semphore = NULL;
ROSProtocol ros_protocol(nullptr, &UsbPort::Instance());

//上下位机通信
PcCom pc_com(UsbPort::Instance());
uint8_t comServiceInit() {
  // can外设初始化
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan1);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan2);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan3);

  // can 总线初始化
  fdcan1_bus.init();
  fdcan2_bus.init();
  fdcan3_bus.init();

  chassis_motor1.init();
  chassis_motor2.init();
  chassis_motor3.init();
  chassis_motor4.init();

  arm2006_motor.init();
  arm3508_motor.init();

//尾部电机的初始化
  tail_claw_move_motor.init();
  tail_claw_roll_motor.init();
  
  lift_2006_motor1.init();
  lift_2006_motor2.init();
  lift_3508_motor1.init();
  lift_3508_motor2.init();

  fdcan3_bus.registerDevice(&chassis_motor1);
  fdcan3_bus.registerDevice(&chassis_motor2);
  fdcan3_bus.registerDevice(&chassis_motor3);
  fdcan3_bus.registerDevice(&chassis_motor4);


  fdcan2_bus.registerDevice(&arm2006_motor);
  fdcan2_bus.registerDevice(&arm3508_motor);
  //fdcan2_bus.registerDevice(&arm4310_motor);
  //注册尾部电机
  fdcan2_bus.registerDevice(&tail_claw_move_motor);
  fdcan2_bus.registerDevice(&tail_claw_roll_motor);

  fdcan1_bus.registerDevice(&lift_2006_motor1);
  fdcan1_bus.registerDevice(&lift_2006_motor2);
  fdcan1_bus.registerDevice(&lift_3508_motor1);
  fdcan1_bus.registerDevice(&lift_3508_motor2);
  

  fdcan1_bus.registerDevice(&lift_2006_motor1);
  fdcan1_bus.registerDevice(&lift_2006_motor2);
  fdcan1_bus.registerDevice(&lift_3508_motor1);
  fdcan1_bus.registerDevice(&lift_3508_motor2);
  

  // 串口外设
   uart2_rx_semphore = osSemaphoreNew(1, 0, NULL);
  uart2_port.startRxDmaIdle();
  uart3_rx_semphore = osSemaphoreNew(1, 0, NULL);
  uart3_port.startRxDmaIdle();
  uart6_port.startRxDmaIdle();
 
  // Xbox控制器初始化
  xbox_remote.init();

  // usb 外设
  usbcdc_rx_semphore = osSemaphoreNew(1, 0, NULL);
  ros_protocol.init();
  UsbPort::Instance().SetRxCallback(onUsbRxCb, NULL);
  return 0;
}

// 回调函数
void onUart2RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart2_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart2_rx_semphore);
  }
}

void onUart3RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart3_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart3_rx_semphore);
  }
}

// 红外模块回调
void onUart6RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  infrared_module.UartPortRxCbHandler(data, len);
}

void onUsbRxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && usbcdc_rx_semphore != NULL) {
    (void)osSemaphoreRelease(usbcdc_rx_semphore);
  }
}

//can发送任务
void can1SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;
  uint8_t len = 8;  
  const uint32_t lift_motor_ids[4] = {0x201, 0x202, 0x203, 0x204};

  for (;;) {
    // 一帧固定打包 4 个槽位：0x201~0x204
    pack.id = 0x200; // DJI Group 2

    int16_t commands[4] = {0};
    commands[0] = static_cast<int16_t>(lift_2006_motor1.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(lift_2006_motor2.cmdTrans()); // 0x202
    commands[2] = static_cast<int16_t>(lift_3508_motor1.cmdTrans()); // 0x203
    commands[3] = static_cast<int16_t>(lift_3508_motor2.cmdTrans()); // 0x204
    packDJIMotorCanMsg(pack.id,lift_motor_ids, commands, 4, pack.data, len);
    fdcan1_bus.addCanMsg(pack);
    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

void can2SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;
  const uint32_t arm_motor_ids[4] = {0x201, 0x202, 0x203, 0x204};
  for (;;) {
    pack.id = 0x200; // DJI Group 2
    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};

    // arm motor
    //commands[0] = static_cast<int16_t>(arm2006_motor.cmdTrans()); // 0x205
    //commands[1] = static_cast<int16_t>(arm3508_motor.cmdTrans()); // 0x206
    commands[0] = static_cast<int16_t>(tail_claw_move_motor.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(tail_claw_roll_motor.cmdTrans()); // 0x202
    commands[2] = static_cast<int16_t>(0);
    commands[3] = static_cast<int16_t>(0);
    packDJIMotorCanMsg(pack.id, arm_motor_ids, commands, 4, pack.data, len);
     fdcan2_bus.addCanMsg(pack);

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}


void can3SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;  
  const uint32_t chassis_motor_ids[4] = {0x201, 0x202, 0x203, 0x204};

  for (;;) {
    // 一帧固定打包 4 个槽位：0x201~0x204
    pack.id = 0x200; // DJI Group 2

    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};
    commands[0] = static_cast<int16_t>(chassis_motor1.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(chassis_motor2.cmdTrans()); // 0x202
    commands[2] = static_cast<int16_t>(chassis_motor3.cmdTrans()); // 0x203   
    commands[3] = static_cast<int16_t>(chassis_motor4.cmdTrans()); // 0x204
    packDJIMotorCanMsg(pack.id, chassis_motor_ids, commands, 4, pack.data, len);
    // arm3508_motor.manager_->addCanMsg(pack);
    fdcan3_bus.addCanMsg(pack);
    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

//接收并处理任务
void uart2RxProcessTask(void *argument){
(void)argument;

for (;;) {
  (void)osSemaphoreAcquire(uart2_rx_semphore, osWaitForever);

   UartPort::Packet packet{};
    while (uart2_port.Read(packet)) {
      for (uint16_t i = 0; i < packet.len; ++i) {
        if (hwt101_parser.processByte(packet.data[i])) {
          //g_hwt101_roll_deg = hwt101_parser.rollDeg();
          //g_hwt101_pitch_deg = hwt101_parser.pitchDeg();
          g_hwt101_yaw_deg = hwt101_parser.yawDeg();
          g_hwt101_frame_count = hwt101_parser.frameCount();
        }
      }
    }
}
}

void uart3RxProcessTask(void *argument) {
  (void)argument;
  if(!xbox_data_pub.IsValid()) {
    return;
  }
  for (;;) {
    (void)osSemaphoreAcquire(uart3_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart3_port.Read(packet)) {
      // 逐字节送进Xbox协议解析器
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = xbox_remote.processByte(packet.data[i]);
        if (frame_id != 0) {
          // 帧解析完成，可以在这里获取控制器数据并做业务处理
          const auto &ctrl_data = xbox_remote.getControllerData();
          xbox_msg.btnY = ctrl_data.btnY;
          xbox_msg.btnA = ctrl_data.btnA;
          xbox_msg.btnShare = ctrl_data.btnShare;
          xbox_msg.btnView = ctrl_data.btnSelect;
          xbox_msg.btnMenu = ctrl_data.btnStart;
          xbox_msg.btnXbox = ctrl_data.btnXbox;
          xbox_msg.btnLB = ctrl_data.btnLB;
          xbox_msg.btnRB = ctrl_data.btnRB;
          xbox_msg.btnLS = ctrl_data.btnLS;
          xbox_msg.btnRS = ctrl_data.btnRS;
          xbox_msg.trigLT = ctrl_data.trigLT;
          xbox_msg.trigRT = ctrl_data.trigRT;
          xbox_msg.btnDirUp = ctrl_data.btnDirUp;
          xbox_msg.btnDirDown = ctrl_data.btnDirDown;
          xbox_msg.btnDirLeft = ctrl_data.btnDirLeft;
          xbox_msg.btnDirRight = ctrl_data.btnDirRight;
          xbox_msg.btnB = ctrl_data.btnB;
          xbox_msg.btnX = ctrl_data.btnX;
          xbox_msg.joyLHori = ctrl_data.joyLHori;
          xbox_msg.joyLVert = ctrl_data.joyLVert;
          xbox_msg.joyRHori = ctrl_data.joyRHori;
          xbox_msg.joyRVert = ctrl_data.joyRVert;
          xbox_data_pub.Publish(xbox_msg);
        }
      }
    }
  }
}


/*
void usbCdcProcessTask(void *argument) {
    (void)argument;

    for (;;) {
        (void)osSemaphoreAcquire(usbcdc_rx_semphore, osWaitForever);

        UsbPort::Packet packet{};
        while (UsbPort::Instance().Read(packet)) {
            // 逐个字节解析
            for (uint16_t i = 0; i < packet.len; ++i) {
                NavProtocol::NavCmd cmd;
                if (nav_protocol.processByte(packet.data[i], cmd)) {
                    // 解析成功，生成响应
                    char resp[64] = {0};
                    NavProtocol::buildResponse(cmd, resp, sizeof(resp));
                    // 通过USB发送响应
                    UsbPort::Instance().WriteAsync(reinterpret_cast<uint8_t*>(resp), strlen(resp));
                }
            }
        }
    }
}
*/

// 下面是协议解析和校验算法的实现，基于之前的设计
void PcComTask(void *argument) {
  (void)argument;
  pc_com.init();

  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {
    osSemaphoreAcquire(usbcdc_rx_semphore, 1);
    pc_com.ProcessRx();
    
    pc_com.ProcessTx();
    vTaskDelayUntil(&currentTime, 1);
  }
}
/*
// ============ 原来的ROSProtocol方式（保留备用） ============
void usbCdcProcessTask_Origin(void *argument) {

    (void)argument;

    for (;;) {
        (void)osSemaphoreAcquire(usbcdc_rx_semphore, osWaitForever);

        UsbPort::Packet packet{};
        while (UsbPort::Instance().Read(packet)) {
            for (uint16_t i = 0; i < packet.len; ++i) {
                uint8_t frame_id = ros_protocol.processData(packet.data[i]);
                if (frame_id != 0) {
                    uint8_t rev[64] = {0};
                    memcpy(rev, ros_protocol.getSensorBagData().i16_data,
                           sizeof(ros_protocol.getSensorBagData().i16_data));
                    memcpy(rev + sizeof(ros_protocol.getSensorBagData().i16_data),
                           ros_protocol.getSensorBagData().f_data,
                           sizeof(ros_protocol.getSensorBagData().f_data));
                    UsbPort::Instance().WriteAsync(
                        rev, sizeof(ros_protocol.getSensorBagData()));
                }
            }
        }
    }
}
// ============ 原来的ROSProtocol方式结束 ============
*/
