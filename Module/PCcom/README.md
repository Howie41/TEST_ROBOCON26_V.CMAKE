# PCcom 小电脑通信使用说明

本文档说明当前工程中 `Module/PCcom` 的上/下位机通信协议、板端调用方式、小电脑发包格式，以及常见调试方法。

## 1. 当前通信链路

当前工程中 `PcCom` 在 `APP/robot_com/com_config.cpp` 中创建：

```cpp
PcCom pc_com(UsbPort::Instance());
```

因此当前小电脑与主控板通信走的是 **USB CDC 虚拟串口**。数据流如下：

```text
小电脑 / PC
  -> USB CDC 虚拟串口
  -> UsbPort 接收队列
  -> PcComTask
  -> pc_com.ProcessRx()
  -> packet_manager 解析协议包
  -> PcCom::OnPacket()
  -> 发布到 topic
```

当前示例命令收到后会发布到 topic：

```cpp
"pc_tail_claw_pub"
```

如果板端需要把消息再转发到某个串口，例如 UART2，可以在任务中订阅这个 topic，然后调用 `uart2_port.write()`。

## 2. 初始化要求

`PcComTask` 启动后必须先调用一次：

```cpp
pc_com.init();
```

否则 `packet_manager` 的发送函数和接收回调不会被设置，协议包即使被读到，也不会进入 `PcCom::OnPacket()`。

推荐结构：

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

USB 设备初始化建议只保留一次 `MX_USB_DEVICE_Init()`，不要在 `main.c` 和 `freertos.c` 中重复初始化。

## 3. 协议帧格式

协议定义在：

```text
Module/transfer_protocol/transfer_protocol.hpp
```

完整数据帧格式：

```text
AA 55 | size_hi size_lo | code_hi code_lo | crc_hi crc_lo | body... | 55 AA
```

字段说明：

| 字段 | 长度 | 字节序 | 说明 |
| --- | --- | --- | --- |
| header | 2 字节 | 固定 | 包头，固定为 `AA 55` |
| size | 2 字节 | 大端 | 整包长度，包含 header、size、code、crc、body、tail |
| code | 2 字节 | 大端 | 命令码 |
| crc | 2 字节 | 大端 | CRC16 校验值 |
| body | N 字节 | 按双方约定 | 数据体 |
| tail | 2 字节 | 固定 | 包尾，固定为 `55 AA` |

最短空包长度是 10 字节：

```text
2 + 2 + 2 + 2 + 0 + 2 = 10
```

当前示例消息 `tail_claw_msg` 的 body 长度是 2 字节，所以整包长度是 12 字节：

```text
0x000C
```

## 4. 当前已有命令

命令码定义在：

```text
Module/PCcom/PCcom.hpp
```

当前已有命令：

```cpp
enum class PcCmd : uint16_t {
  tail_claw_msg = 0x0001,
};
```

对应消息结构体定义在：

```text
APP/robot_task/topic_pool.h
```

当前结构体：

```cpp
struct tail_claw_msg {
  uint16_t distance;
};
```

注意：`body` 是直接 `memcpy` 到结构体里的原始字节。STM32 是小端机器，所以 `distance` 的 body 使用小端序。

例如：

```text
distance = 100 = 0x0064
body = 64 00
```

## 5. CRC16 算法

当前使用：

```cpp
gdut::crc16_algorithm
```

参数：

```text
初始值: 0xFFFF
多项式: 0xA001 反向形式
计算范围: 整包中除 crc 两个字节以外的所有字节
写入顺序: crc 高字节在前，低字节在后
```

也就是说计算 CRC 时参与的数据为：

```text
header + size + code + body + tail
```

不包含 `crc_hi crc_lo` 本身。

## 6. 小电脑发送示例

发送 `tail_claw_msg{ distance = 100 }`：

```text
AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

含义：

```text
AA 55      header
00 0C      size = 12
00 01      code = tail_claw_msg
29 C4      crc16
64 00      body: distance = 100
55 AA      tail
```

Python 发送示例：

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
body = distance.to_bytes(2, "little")
frame = build_packet(0x0001, body)

print(frame.hex(" ").upper())
ser.write(frame)
```

输出应为：

```text
AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

## 7. 使用串口助手 / VOFA 测试

如果使用 VOFA 或普通串口助手向 USB CDC 发送数据：

1. 选择 STM32 的 USB 虚拟串口。
2. 必须使用 **HEX / 十六进制发送**。
3. 不要用文本模式发送 `AA 55 00 ...`，否则板子收到的是 ASCII 字符，而不是二进制字节。

错误示例：

```text
文本发送: "AA 55 00 0C ..."
```

板子实际收到：

```text
41 41 20 35 35 20 ...
```

正确示例：

```text
HEX 发送: AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

板子实际收到：

```text
AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

## 8. 板端接收使用

当前 `PcCom::OnPacket()` 中收到 `tail_claw_msg` 后会发布：

```cpp
pc_tail_claw_pub_.Publish(msg);
```

业务任务中可以这样订阅：

```cpp
static TypedTopicSubscriber<tail_claw_msg> tail_claw_subscriber(
    "pc_tail_claw_pub", 8);

tail_claw_msg msg{};
if (tail_claw_subscriber.TryGet(&msg)) {
  // 使用 msg.distance
}
```

如果只是想验证 UART2 输出，可以临时打印 ASCII：

```cpp
char buf[32];
int len = snprintf(buf, sizeof(buf), "distance=%u\r\n", msg.distance);
if (len > 0) {
  uart2_port.write(reinterpret_cast<const uint8_t *>(buf),
                   static_cast<size_t>(len), 100);
}
```

UART2 当前配置为：

```text
115200, 8N1
TX: PD5
RX: PD6
```

接 USB-TTL 时：

```text
STM32 PD5 / USART2_TX -> USB-TTL RX
STM32 GND             -> USB-TTL GND
```

## 9. 板端发送给小电脑

`PcCom::ProcessTx()` 会订阅：

```cpp
"pc_tail_claw_sub"
```

如果某个任务想主动向小电脑发送 `tail_claw_msg`，可以发布：

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

会把消息封装成协议包并通过 USB 发给小电脑。

## 10. 新增命令流程

新增一类小电脑消息时，通常需要改 3 个位置。

第一步，在 `APP/robot_task/topic_pool.h` 中定义结构体：

```cpp
struct my_msg {
  uint16_t value;
  uint8_t mode;
};
```

第二步，在 `Module/PCcom/PCcom.hpp` 中增加命令码：

```cpp
enum class PcCmd : uint16_t {
  tail_claw_msg = 0x0001,
  my_msg = 0x0002,
};
```

第三步，在 `Module/PCcom/PCcom.cpp` 中增加解析逻辑：

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

如果还需要板端主动发送，也要在 `ProcessTx()` 中增加对应 topic 订阅和 `send()` 调用。

## 11. 常见问题排查

### 11.1 电脑看不到 USB 虚拟串口

可能原因：

1. 程序没有运行。
2. 程序卡在 `MX_USB_DEVICE_Init()` 前。
3. USB 初始化被重复调用。
4. USB 线不是数据线。
5. USB 时钟或硬件连接异常。

### 11.2 USB 收到了，但协议不进 `OnPacket`

重点检查：

1. 是否 HEX 发送。
2. `size` 是否是整包长度，不是 body 长度。
3. `code` 是否正确。
4. CRC 是否正确。
5. `tail` 是否是 `55 AA`。
6. `pc_com.init()` 是否已经调用。

### 11.3 进了 `OnPacket`，但是业务没有反应

重点检查：

1. 是否订阅了正确 topic：`"pc_tail_claw_pub"`。
2. `body_size()` 是否等于 `sizeof(tail_claw_msg)`。
3. 业务任务是否在正常运行。
4. 如果通过 UART2 验证，确认 UART2 接线和波特率。

## 12. 当前示例快速测试

小电脑发送：

```text
AA 55 00 0C 00 01 29 C4 64 00 55 AA
```

板端成功解析后：

```cpp
msg.distance == 100
```

如果 `uart2RxProcessTask` 中使用 ASCII 打印，应在 UART2 看到：

```text
distance=100
```

