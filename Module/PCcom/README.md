# PCcom USB 小电脑通信说明书

本文说明当前工程中主控板通过 USB CDC 虚拟串口与小电脑通信的实现、使用方式、信息流，以及当前静态检查发现的问题。

## 1. 当前通信链路

当前小电脑通信对象在 `APP/robot_com/com_config.cpp` 中创建：

```cpp
PcCom pc_com(UsbPort::Instance());
```

因此当前走的是 USB CDC 虚拟串口，不是 UART。

接收方向的信息流：

```text
小电脑
  -> USB CDC 虚拟串口
  -> USB_DEVICE/App/usbd_cdc_if.c 的 CDC_Receive_HS()
  -> UsbPort_OnRxFromIsr()
  -> UsbPort::OnRxFromIsr()
  -> UsbPort 接收队列 rx_queue_
  -> PcComTask
  -> pc_com.ProcessRx()
  -> gdut::packet_manager::receive()
  -> PcCom::OnPacket()
  -> 发布到 topic
  -> 业务任务订阅并执行
```

发送方向的信息流：

```text
业务任务
  -> 发布到发送 topic
  -> PcComTask
  -> pc_com.ProcessTx()
  -> PcCom::send()
  -> gdut::data_packet 封包
  -> gdut::packet_manager::send()
  -> UsbPort::WriteAsync()
  -> UsbPort 发送队列 tx_queue_
  -> UsbPort::PumpTx()
  -> CDC_Transmit_HS()
  -> USB CDC
  -> 小电脑
```

## 2. 相关文件分工

| 文件 | 作用 |
| --- | --- |
| `Module/PCcom/PCcom.hpp/.cpp` | 上下位机通信模块，负责协议封包、解包、命令分发和 topic 桥接 |
| `Module/transfer_protocol/transfer_protocol.hpp` | 通用二进制帧协议，提供 `data_packet` 和 `packet_manager` |
| `Module/transfer_protocol/verification_algorithm.hpp` | CRC16 校验算法 |
| `BSP/UsbPort.hpp/.cpp` | USB CDC 的 C++ 封装，提供收发队列和异步发送 |
| `USB_DEVICE/App/usbd_cdc_if.c` | STM32 USB CDC 回调入口 |
| `APP/robot_com/com_config.cpp` | 创建 `pc_com`、初始化 USB 回调、启动 `PcComTask` |
| `Module/topics/topics.hpp/.cpp` | 板内 publish/subscribe 消息总线 |
| `APP/robot_task/topic_pool.h` | topic 消息结构体定义 |

## 3. 初始化要求

`PcComTask` 中必须先调用：

```cpp
pc_com.init();
```

这个函数会给 `packet_manager` 注册两个回调：

```cpp
manager_.set_send_function(...);
manager_.set_receive_function(...);
```

如果不调用 `init()`，收到 USB 数据后不会进入 `PcCom::OnPacket()`，发送时也不会真正写入 USB。

当前 `PcComTask` 主循环如下：

```cpp
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
```

这里的逻辑是：USB 接收回调释放 `usbcdc_rx_semphore`，任务醒来后把 `UsbPort` 队列里的数据交给协议解析器；即使没有收到数据，也会每 1 ms 执行一次 `ProcessTx()` 检查是否有板端要发给小电脑的数据。

## 4. 协议帧格式

协议定义在 `Module/transfer_protocol/transfer_protocol.hpp`。

完整帧格式：

```text
AA 55 | size_hi size_lo | code_hi code_lo | crc_hi crc_lo | body... | 55 AA
```

| 字段 | 长度 | 字节序 | 含义 |
| --- | --- | --- | --- |
| header | 2 字节 | 固定值 | 包头，固定为 `AA 55` |
| size | 2 字节 | 大端 | 整包长度，包含 header、size、code、crc、body、tail |
| code | 2 字节 | 大端 | 命令码 |
| crc | 2 字节 | 大端 | CRC16 校验值 |
| body | N 字节 | 按结构体原始内存 | 数据体 |
| tail | 2 字节 | 固定值 | 包尾，固定为 `55 AA` |

最短空包长度：

```text
2 + 2 + 2 + 2 + 0 + 2 = 10 字节
```

当前示例消息 `tail_claw_msg` 的 body 是：

```cpp
struct tail_claw_msg {
  int16_t distance;
};
```

所以它的整包长度是 12 字节。

## 5. 当前命令码

命令码定义在 `Module/PCcom/PCcom.hpp`：

```cpp
enum class PcCmd : uint16_t {
  tail_claw_msg = 0x0001,
  tail_claw_msg_flase = 0x0002,
  tail_claw_msg_success = 0x0003,
};
```

当前真正实现了解析和发送的只有：

```text
0x0001: tail_claw_msg
```

`tail_claw_msg_flase` 和 `tail_claw_msg_success` 目前只是枚举值，还没有在 `OnPacket()` 或 `ProcessTx()` 中实现业务逻辑。

## 6. CRC16 规则

当前使用 `gdut::crc16_algorithm`：

```cpp
using Packet = gdut::data_packet<gdut::crc16_algorithm>;
using Manager = gdut::packet_manager<gdut::crc16_algorithm>;
```

规则：

```text
初始值: 0xFFFF
查表算法等价于 CRC16/MODBUS 反向多项式 0xA001
计算范围: 整包中除 crc 两个字节以外的所有字节
写入顺序: crc 高字节在前，低字节在后
```

也就是参与计算的数据为：

```text
header + size + code + body + tail
```

不包含 `crc_hi crc_lo` 本身。

## 7. 小电脑发包示例

发送 `tail_claw_msg{distance = 100}`：

```text
AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

解释：

```text
AA 55      header
00 0C      size = 12
00 01      code = tail_claw_msg
29 C4      crc16
64 00      body，int16_t distance = 100，小端
55 AA      tail
```

Python 示例：

```python
import serial

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc

def build_packet(code: int, body: bytes) -> bytes:
    size = 2 + 2 + 2 + 2 + len(body) + 2

    packet = bytearray()
    packet += bytes([0xAA, 0x55])
    packet += size.to_bytes(2, "big")
    packet += code.to_bytes(2, "big")
    packet += b"\x00\x00"
    packet += body
    packet += bytes([0x55, 0xAA])

    crc = crc16_modbus(packet[:6] + packet[8:])
    packet[6] = (crc >> 8) & 0xFF
    packet[7] = crc & 0xFF
    return bytes(packet)

ser = serial.Serial("COM20", 115200, timeout=0.1)

distance = 100
body = int(distance).to_bytes(2, "little", signed=True)
frame = build_packet(0x0001, body)

print(frame.hex(" ").upper())
ser.write(frame)
```

注意：USB CDC 虚拟串口里设置的波特率通常只是主机侧串口软件参数，对 USB 本身不决定真实传输速率；但串口助手仍然需要选对 COM 口。

## 8. 串口助手测试方法

使用 VOFA 或普通串口助手时：

1. 选择 STM32 枚举出的 USB 虚拟串口。
2. 使用 HEX/十六进制发送。
3. 不要用文本模式发送 `AA 55 00 ...`。

错误方式：

```text
文本发送: "AA 55 00 0C ..."
```

板子实际收到的是 ASCII：

```text
41 41 20 35 35 20 ...
```

正确方式：

```text
HEX 发送: AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

板子实际收到：

```text
AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

## 9. 板端接收使用方式

当前 `PcCom::OnPacket()` 收到 `tail_claw_msg` 后：

```cpp
tail_claw_msg msg{};
std::memcpy(&msg, packet.body_data(), sizeof(tail_claw_msg));
pc_tail_claw_pub_.Publish(msg);
```

它会发布到 topic：

```text
pc_tail_claw_pub
```

业务任务订阅方式：

```cpp
static TypedTopicSubscriber<tail_claw_msg> tail_claw_subscriber(
    "pc_tail_claw_pub", 8);

tail_claw_msg msg{};
if (tail_claw_subscriber.TryGet(&msg)) {
  // 使用 msg.distance
}
```

当前 `APP/tail_claw_task/tail_claw_task.cpp` 已经这样订阅，并根据 `distance` 控制尾部横移方向：

```text
distance < -5: 左移
distance >  5: 右移
否则: 不横移
```

## 10. 板端发送给小电脑

`PcCom::ProcessTx()` 当前订阅：

```text
pc_tail_claw_sub
```

业务任务如果要主动发 `tail_claw_msg` 给小电脑，可以发布：

```cpp
static TypedTopicPublisher<tail_claw_msg> tail_claw_pc_pub(
    "pc_tail_claw_sub");

tail_claw_msg msg{};
msg.distance = 100;
tail_claw_pc_pub.Publish(msg);
```

随后 `PcComTask` 中的：

```cpp
pc_com.ProcessTx();
```

会把它封装成协议包，通过 USB CDC 发给小电脑。

## 11. 新增命令流程

新增一类小电脑消息通常需要改四处。

第一步，在 `APP/robot_task/topic_pool.h` 定义结构体：

```cpp
struct my_msg {
  int16_t value;
  uint8_t mode;
};
```

建议尽量使用固定宽度类型，例如 `uint8_t`、`int16_t`、`float`。如果协议要跨平台长期使用，不建议直接传带 `bool` 或隐式 padding 的结构体。

第二步，在 `Module/PCcom/PCcom.hpp` 增加命令码：

```cpp
enum class PcCmd : uint16_t {
  tail_claw_msg = 0x0001,
  my_msg = 0x0002,
};
```

第三步，在 `PcCom` 中增加对应 publisher：

```cpp
TypedTopicPublisher<my_msg> my_msg_pub_{"pc_my_msg_pub"};
```

第四步，在 `Module/PCcom/PCcom.cpp` 的 `OnPacket()` 中增加解析：

```cpp
case static_cast<uint16_t>(PcCmd::my_msg): {
  if (packet.body_size() != sizeof(my_msg)) {
    return;
  }

  my_msg msg{};
  std::memcpy(&msg, packet.body_data(), sizeof(my_msg));
  my_msg_pub_.Publish(msg);
  break;
}
```

如果还需要板端主动发给小电脑，再在 `ProcessTx()` 中订阅发送 topic 并调用：

```cpp
send(static_cast<uint16_t>(PcCmd::my_msg), msg);
```

## 12. 常见问题排查

### 12.1 小电脑看不到 USB 虚拟串口

重点检查：

1. USB 数据线是否支持数据传输。
2. 程序是否卡在 `MX_USB_DEVICE_Init()` 或更早的位置。
3. `MX_USB_DEVICE_Init()` 是否被重复调用。
4. USB 时钟、GPIO、OTG HS/FS 配置是否正确。
5. Windows 设备管理器是否枚举为 CDC 设备。

### 12.2 UsbPort 收到了数据，但没有进入 OnPacket

重点检查：

1. 是否使用 HEX 发送。
2. `size` 是否是整包长度，不是 body 长度。
3. `code` 是否与 `PcCmd` 一致。
4. CRC 是否按当前规则计算。
5. 包尾是否是 `55 AA`。
6. `pc_com.init()` 是否已经执行。

### 12.3 进入 OnPacket，但业务没有动作

重点检查：

1. 业务任务是否订阅了正确 topic，例如 `pc_tail_claw_pub`。
2. body 长度是否等于 `sizeof(对应结构体)`。
3. 业务任务是否已经启动。
4. topic 池容量是否够用。
5. 消息是否被业务任务每周期清零或覆盖。

## 13. 静态检查发现的问题和风险

### 高风险 1：USB 初始化被调用了两次

当前 `Core/Src/main.c` 中调用了一次：

```cpp
MX_USB_DEVICE_Init();
```

但 `Core/Src/freertos.c` 的 `StartDefaultTask()` 中又调用了一次：

```cpp
MX_USB_DEVICE_Init();
```

USB Device 通常不应该重复初始化。建议只保留一次。当前代码注释也写了希望放在 `main.c` 靠前位置，因此更建议删除或注释 `freertos.c` 里的那一次。

### 高风险 2：RTOS 初始化顺序可疑

当前 `main.c` 中先调用：

```cpp
Robot_Init();
```

`Robot_Init()` 内部会创建 semaphores 和 threads：

```cpp
comServiceInit();
osTaskInit();
```

但 `osKernelInitialize()` 在 `Robot_Init()` 之后才调用。CMSIS-RTOS2 的推荐流程是：

```text
osKernelInitialize()
创建 RTOS 对象和线程
osKernelStart()
```

建议把 `Robot_Init()` 中依赖 RTOS 对象创建的部分放到 `osKernelInitialize()` 之后执行，或者把 `Robot_Init()` 拆成硬件初始化和 RTOS 对象初始化两段。

### 高风险 3：USB 接收回调名字带 FromIsr，但调用了普通 osSemaphoreRelease

`UsbPort::OnRxFromIsr()` 会调用 `onUsbRxCb()`，里面执行：

```cpp
osSemaphoreRelease(usbcdc_rx_semphore);
```

如果该回调确实处在 USB 中断上下文，使用 CMSIS 封装是否安全要结合当前 FreeRTOS/CMSIS 配置确认。更稳妥的方式是使用明确的 ISR-safe 通知机制，或者确认 `osSemaphoreRelease` 在当前移植层允许从 ISR 调用。

### 中风险 4：UsbPort 发送状态变量有中断/任务并发风险

`UsbPort::PumpTx()` 在任务里调用，`UsbPort::OnTxCpltFromIsr()` 在 USB 发送完成回调中调用。二者共享：

```cpp
tx_staging_
tx_staging_valid_
tx_inflight_
```

其中只有 `tx_inflight_` 是 `volatile bool`，不是原子量，也没有临界区保护。轻载下通常能跑，但在连续高频发送时可能出现状态竞争。建议用临界区或原子状态机保护发送状态。

### 中风险 5：协议接收缓冲没有最大长度限制

`packet_manager::receive()` 会持续向 `m_receive_buffer` 追加数据。如果小电脑发送了一个合法包头但 size 很大，且后续一直不补齐完整包，缓冲区可能持续增长。建议加最大包长限制，例如 256 或 512 字节，超过后丢弃并重新同步。

### 中风险 6：直接 memcpy 结构体作为协议 body，可移植性较差

当前 body 直接是 MCU 内存布局：

```cpp
std::memcpy(&msg, packet.body_data(), sizeof(tail_claw_msg));
```

这要求小电脑端完全遵守：

```text
字段类型一致
大小端一致
结构体 padding 一致
float 格式一致
bool 大小一致
```

当前 `tail_claw_msg` 只有一个 `int16_t`，问题不大；后续复杂结构体建议手动序列化每个字段。

### 中风险 7：`PcCom` 里有一个未使用的成员订阅者

`PCcom.hpp` 中定义了：

```cpp
TypedTopicSubscriber<tail_claw_msg> pc_tail_claw_sub_{"pc_tail_claw_sub",8};
```

但 `ProcessTx()` 里又定义了一个同 topic 的 static subscriber：

```cpp
static TypedTopicSubscriber<tail_claw_msg> tail_claw_subscriber(
    "pc_tail_claw_sub", 8);
```

成员 `pc_tail_claw_sub_` 当前未使用，还会额外占用一个订阅者名额。建议删掉成员，或者改为在 `ProcessTx()` 使用这个成员。

### 低风险 8：命名拼写错误

`tail_claw_msg_flase` 应该是 `false`。如果后续小电脑协议也按这个名字生成代码，容易造成误解。

### 低风险 9：当前说明和注释存在编码乱码

多处中文注释显示为乱码，不影响编译，但会影响维护。建议统一保存为 UTF-8。

## 14. 推荐整改优先级

1. 先修 USB 重复初始化。
2. 再理顺 `osKernelInitialize()`、RTOS 对象创建、`osKernelStart()` 的顺序。
3. 给 `packet_manager` 增加最大包长保护。
4. 清理 `PcCom` 中未使用的订阅者成员。
5. 后续新增复杂消息时，改成显式序列化，而不是直接 memcpy 结构体。

