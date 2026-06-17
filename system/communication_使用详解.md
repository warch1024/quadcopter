# Communication（HU-M40 遥控协议）使用详解

## 一、概述

`communication.c` 实现了飞控与 **HU-M40 遥控器**之间的无线通信协议。它基于 NRF24L01 硬件平台，在 Enhanced ShockBurst 协议之上，定义了一套**应用层协议**，包含：

- **16 字节固定长度数据包**的收发
- **自动地址配对（MAC 交换）**：飞控与遥控器自动协商通信地址
- **完整性校验**：校验和（checksum）验证
- **超时断连检测**：2 秒无数据判定为断开

---

## 二、数据包格式（应用层）

### 2.1 整体结构（16 字节）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | 帧头 | 固定 `0x01` |
| 1 | 1 | 功能码 | 遥控器发送 `0x03`，飞控 ACK 返回 `0x83` |
| 2 | 1 | 序号 | 数据包序列号，用于配对时的关联 |
| 3 | 1 | 数据长度 | 有效数据长度，飞控回传 ACK 时为 `11` |
| 4 | 1 | MAC 字节 0 | 地址域（用于地址配对） |
| 5 | 1 | MAC 字节 1 | 地址域 |
| 6 | 1 | MAC 字节 2 | 地址域 |
| 7 | 1 | MAC 字节 3 | 地址域 |
| 8 | 1 | MAC 字节 4 | 地址域 |
| 9~14 | 6 | 保留 / 数据 | 未使用的填充字节 |
| 15 | 1 | 校验和 | 前 15 字节累加和 |

### 2.2 遥控器发送数据

飞控端收到的 16 字节数据（`g_rc_data`）的布局：

| 偏移 | 名称 | 说明 |
|------|------|------|
| 0 | 帧头 | `0x01` |
| 1 | 功能码 | `0x03` |
| 2 | 序号 | 递增序号 |
| 3 | 保留 | - |
| **4** | **HU_RX** | **右摇杆 X 轴**（0~255，中值 0x80） |
| **5** | **HU_RY** | **右摇杆 Y 轴**（0~255，中值 0x80） |
| **6** | **HU_LX** | **左摇杆 X 轴**（0~255，中值 0x80） |
| **7** | **HU_LY** | **左摇杆 Y 轴**（0~255，中值 0x80） |
| **8** | 按键低字节 | 6 个按键的 bit 位（见下方） |
| 9 | 按键高字节 | 扩展按键位 |
| 10~14 | 保留/其他数据 | - |
| 15 | 校验和 | 前 15 字节累加和 |

### 2.3 按键位映射

`g_rc_data[8]` 的低 6 位对应 6 个按键：

| 宏 | 值 | 说明 |
|-----|------|------|
| `HU_K1` | `0x01` | 按键 1（bit0） |
| `HU_K2` | `0x02` | 按键 2（bit1） |
| `HU_K3` | `0x04` | 按键 3（bit2） |
| `HU_K4` | `0x08` | 按键 4（bit3） |
| `HU_K5` | `0x10` | 按键 5（bit4） |
| `HU_K6` | `0x20` | 按键 6（bit5） |

使用方式：

```c
if (g_rc_data[8] & HU_K1) {
    // 按键 1 按下
}
// 或者用 buttons 变量一次性读取
uint16_t buttons = g_rc_data[8] | ((uint16_t)g_rc_data[9] << 8);
```

### 2.4 摇杆值约定

| 摇杆 | 偏移 | 最小值 | 中值 | 最大值 |
|------|------|--------|------|--------|
| HU_RX（右 X） | 4 | 0x00 | **0x80** | 0xFF |
| HU_RY（右 Y） | 5 | 0x00 | **0x80** | 0xFF |
| HU_LX（左 X） | 6 | 0x00 | **0x80** | 0xFF |
| HU_LY（左 Y） | 7 | 0x00 | **0x80** | 0xFF |

- 中值 `0x80`（128）代表摇杆回中
- 可用值范围：`0x00` 到 `0xFF`（0~255）

---

## 三、地址配对协议（核心机制）

HU-M40 协议实现了**自动地址配对**，使得飞控端无需预知遥控器的通信地址。

### 3.1 地址定义

```c
#define HU_M40_ADDR_DEF  "HXFB0"   // 默认广播地址（5 字节）
#define HU_M40_ADDR_ACK  "HXFB1"   // 飞控自身 MAC 地址（5 字节）
```

- **`HXFB0`**：遥控器默认使用的发送地址 + 飞控初始监听地址
- **`HXFB1`**：飞控自身的 MAC 地址，会在 ACK 包中传递给遥控器

### 3.2 配对流程

```
飞控端 (STM32)                  HU-M40 遥控器
    │                                │
    │ 1. RX_Mode("HXFB0", "HXFB0")   │  监听默认地址
    │◄──────── 摇杆/按键数据 ─────────│  遥控器发送帧头0x01+功能码0x03
    │                                │
    │ 2. 验证数据合法                 │
    │    dat[0]==0x01                 │
    │    dat[1]==0x03                 │
    │    dat[15]==checksum(dat,15)    │
    │                                │
    │ 3. TX_Mode("HXFB0", "HXFB0")   │  切换为发送模式
    │ ──────── ACK 包 ───────────────►│  回复帧头0x01+功能码0x83+飞控MAC
    │                                │
    │ 4. RX_Mode("HXFB1", "HXFB1")   │  切换到自身的 MAC 地址监听
    │    g_tx_addr = "HXFB1"         │
    │◄──── 遥控器切换到飞控MAC ───────│  遥控器改用飞控MAC作为发送地址
    │        持续数据收发              │
    │                                │
```

**详细步骤**：

1. **飞控初始化**：以默认地址 `HXFB0` 进入接收模式
2. **收到遥控器数据**：校验帧头 `0x01`、功能码 `0x03` 和校验和
3. **第一次配对**：发送 ACK 包，包含飞控的 MAC 地址 `HXFB1`
4. **切换接收地址**：飞控切换到自己的 MAC 地址 `HXFB1` 监听
5. **遥控器切换发送地址**：遥控器收到 ACK 后，提取飞控 MAC，切换到该地址发送

> 此后，飞控持续监听 `HXFB1`，遥控器持续向 `HXFB1` 发送数据。

### 3.3 ACK 包结构

飞控回复的 ACK 包（16 字节）：

| 偏移 | 值 | 说明 |
|------|-----|------|
| 0 | `0x01` | 帧头 |
| 1 | `0x83` | 功能码（0x80 | 0x03，回传应答标志） |
| 2 | dat[2] | 回传遥控器数据的序号 |
| 3 | `11` | 数据长度 |
| 4~8 | `g_rx_addr[0..4]` | 飞控 MAC 地址（HXFB1） |
| 9~14 | `0` | 填充 |
| 15 | checksum | 前 15 字节累加和 |

### 3.4 断连检测与重连

```c
if (connected && (tick_count - last_rx_tick > 2000 / portTICK_PERIOD_MS)) {
    connected = 0;
    g_rc_connected = 0;
    // 恢复到默认地址监听，等待重新配对
    NRF24L01_RX_Mode(HU_M40_ADDR_DEF, HU_M40_ADDR_DEF);
}
```

- **超时时间**：2 秒（FreeRTOS tick 单位）
- 每收到一包合法数据，更新 `last_rx_tick`
- 超过 2 秒未收到数据，判定为断开
- 断开后自动恢复到默认地址 `HXFB0` 监听，等待遥控器重新连接

---

## 四、各函数详解

### 4.1 `hu_m40_checksum()` — 校验和计算

```c
static uint8_t hu_m40_checksum(uint8_t *buf, uint8_t len)
```

对缓冲区前 `len` 个字节做累加求和，返回 8 位累加结果。飞控在接收数据时校验，发送 ACK 时也计算。

### 4.2 `hu_m40_receive()` — 接收数据

```c
static uint8_t hu_m40_receive(uint8_t *buf)
```

封装 `NRF24L01_RxPacket()`。收到数据返回 1，无数据返回 0。

### 4.3 `hu_m40_send_ack()` — 发送应答

```c
static void hu_m40_send_ack(uint8_t *dat)
```

构建 ACK 包并发送。流程：

1. 构建 16 字节 ACK 帧
2. 填充帧头 `0x01`、功能码 `0x83`、序号、飞控 MAC
3. 计算校验和填充字节 15
4. **切换为 TX_Mode**：`NRF24L01_TX_Mode(g_tx_addr, g_tx_addr)`
5. 发送 ACK 包：`NRF24L01_TxPacket(ack, 16)`

> **注意**：发送完 ACK 后，由调用方负责切回 RX_Mode。

### 4.4 `vTaskCommunicationTest()` — 通信任务主循环

```c
void vTaskCommunicationTest(void *pvParameters)
```

这是一个 **FreeRTOS 任务**，整体流程：

```
NRF24L01_Init()
       ↓
RX_Mode(HXFB0)
       ↓
while (1):
    if 收到数据:
        → 校验帧头、功能码、checksum
        → 更新 last_rx_tick
        → 如果是首次连接，打印连接信息
        → 复制数据到 g_rc_data（全局共享）
        → 发送 ACK（含 MAC）
        → 切回 RX_Mode(g_rx_addr)
        → 更新 g_tx_addr

    if 已连接 && 超时 2 秒:
        → 标记断开
        → 恢复默认地址监听

    vTaskDelay(10ms)
```

---

## 五、全局变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `g_rc_data[16]` | `uint8_t[16]` | 最新收到的遥控器数据，供其他模块读取 |
| `g_rc_connected` | `uint8_t` | 连接状态标志，`1`=已连接，`0`=已断开 |
| `g_rx_addr[5]` | `uint8_t[5]` | 飞控接收地址（初始为 `HXFB1`） |
| `g_tx_addr[5]` | `uint8_t[5]` | 飞控发送/监听地址（初始为 `HXFB0`，配对后更新） |

**其他模块如何使用**：

```c
// 在 electronic_speed_controller.c 中使用
if (g_rc_connected) {
    ly = g_rc_data[HU_LY];  // 左摇杆 Y（油门）
    ry = g_rc_data[HU_RY];  // 右摇杆 Y
    rx = g_rc_data[HU_RX];  // 右摇杆 X（偏航）
}
```

---

## 六、如何添加新的通信协议

如果你需要基于 NRF24L01 实现自定义的通信协议，可以参考以下几点：

### 6.1 数据包结构

```c
#define CUSTOM_PLOAD_WIDTH  16   // 推荐 16 或 32 字节

typedef struct {
    uint8_t  header;       // 帧头，用于识别起始
    uint8_t  cmd;          // 命令码
    uint8_t  seq;          // 序号（可选，用于防丢包）
    uint8_t  data[12];     // 有效数据
    uint8_t  checksum;     // 校验和
} custom_packet_t;
```

### 6.2 基本读写流程

```c
// 接收端
NRF24L01_RX_Mode(my_rx_addr, my_tx_addr);
while (1) {
    if (NRF24L01_RxPacket(buf)) {
        // 处理收到的数据
    }
    vTaskDelay(10);
}

// 发送端
NRF24L01_TX_Mode(peer_rx_addr, my_tx_addr);
NRF24L01_TxPacket(buf, len);
```

### 6.3 地址管理注意事项

- 每对通信设备需要唯一的 5 字节地址
- TX_ADDR（发送端）必须等于 RX_ADDR_P0（接收端）
- 发送端的 RX_ADDR_P0 应设为与 TX_ADDR 相同——这是基于 NRF24L01 ACK 机制的要求

---

## 七、数据流全景

```
HU-M40 遥控器
     │
     │  NRF24L01 (2.4GHz, 2Mbps, CH=25)
     ▼
NRF24L01_RxPacket()           ← communication.c
     │
     ▼
hu_m40_receive()              ← 接收原始 16 字节
     │
     ▼
校验帧头(0x01)/功能码(0x03)/Checksum
     │
     ├── 校验失败 → 丢弃
     │
     └── 校验成功
           │
           ├── memcpy → g_rc_data[16]    ← 全局共享
           │
           ├── hu_m40_send_ack()          ← 回复 ACK 含 MAC
           │
           └── NRF24L01_RX_Mode(新地址)    ← 切换监听地址
                 │
                 ▼
          g_rc_data 被其他模块使用
                 │
     ┌───────────┼───────────┐
     ▼           ▼           ▼
  electronic_speed_controller.c
  attitude.c
  main.c (其他任务)
```

---

## 八、常见问题

### Q1：收不到遥控器数据

- **检查 NRF24L01 是否初始化成功**：`NRF24L01_Check()` 返回 0
- **射频参数是否匹配**：RF_CH、空中速率、CRC 设置与遥控器一致
- **地址是否匹配**：飞控监听默认地址 `HXFB0`
- **校验超时**：遥控器 2000ms 无回应会自动重新初始化

### Q2：能收到但校验失败

```
dat[0] != 1 || dat[1] != 3 || checksum 不匹配
```

- 确保 NRF24L01 负载长度设为 16 字节
- 检查双方的 CRC 设置一致（2 字节 CRC）
- 确保没有其他 2.4G 设备干扰（可尝试改 RF_CH）

### Q3：连接后频繁断连

- **电源稳定性**：NRF24L01 对电源噪声敏感，确保供电充足
- **天线问题**：检查天线焊接，保持净空
- **距离过远**：本工程发射功率设为 0dBm，2Mbps 速率，空旷距离约 10~30m
- **重发参数**：SETUP_RETR = 0x88（8 次重发，延迟 2250μs）

### Q4：`g_rc_connected` 总是 0

- 启动后需等待遥控器数据到达，`dat[0]==1 && dat[1]==3` 且 checksum 正确后才置 1
- 遥控器需先上电并处于发送模式
- 超时 2 秒后自动清除连接标志

---

## 九、与 NRF24L01 驱动的关系

```
┌──────────────────────────────────────┐
│         communication.c              │ ← 应用层协议
│  HU-M40 协议编解码 / 地址配对 / 校验  │
├──────────────────────────────────────┤
│         NRF24L01.c                   │ ← 硬件抽象层
│  SPI 读写 / 收发模式切换 / ESB 协议   │
├──────────────────────────────────────┤
│         STM32 HAL (SPI1 + GPIO)       │ ← 硬件层
└──────────────────────────────────────┘
```

`communication.c` 位于应用层，不直接操作 SPI 或 NRF24L01 寄存器，所有硬件操作都通过 `NRF24L01.c` 的 API 完成。

---

## 十、使用示例

### 10.1 在 main.c 中创建任务

```c
xTaskCreate(vTaskCommunicationTest, "comm", 256, NULL, 3, NULL);
```

任务栈大小至少 256 字（Words），优先级 3。

### 10.2 在其他任务中读取遥控数据

```c
#include "communication.h"

void vSomeTask(void *pvParameters)
{
    while (1) {
        if (g_rc_connected) {
            uint8_t throttle = g_rc_data[HU_LY];   // 油门（左摇杆 Y）
            uint8_t yaw      = g_rc_data[HU_RX];   // 偏航（右摇杆 X）
            uint8_t pitch    = g_rc_data[HU_RY];   // 俯仰（右摇杆 Y）
            uint8_t roll     = g_rc_data[HU_LX];   // 横滚（左摇杆 X）

            if (g_rc_data[8] & HU_K1) {
                // K1 按下 → 解锁电机
            }
        }
        vTaskDelay(10);
    }
}
```

### 10.3 完整的初始化调用顺序

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();

    // 创建通信任务（内部完成 NRF24L01 初始化和配对）
    xTaskCreate(vTaskCommunicationTest, "comm", 256, NULL, 3, NULL);

    // 创建其他任务（电调控制、姿态解算等）
    // ...

    vTaskStartScheduler();  // 启动调度器
}
```

---

## 十一、配置总结

| 参数 | 值 | 说明 |
|------|-----|------|
| 数据包大小 | 16 字节 | 固定长度 |
| 默认地址 | `HXFB0` | 5 字节 ASCII |
| ACK 地址 | `HXFB1` | 飞控自身 MAC |
| 帧头 | `0x01` | 数据包起始识别 |
| 遥控器功能码 | `0x03` | 数据帧 |
| ACK 功能码 | `0x83` | 应答帧 |
| 校验方式 | 累加和 checksum | 前 15 字节求和 |
| 断连超时 | 2 秒 | 超过 2 秒无数据视为断开 |
| 任务周期 | 10ms | FreeRTOS vTaskDelay |
| 摇杆中值 | `0x80` | 128 |
| 按键数量 | 6 个 | HU_K1 ~ HU_K6 |
| RF 通道 | 25 | 2.425GHz |
| 空中速率 | 2Mbps | NRF24L01 配置 |
