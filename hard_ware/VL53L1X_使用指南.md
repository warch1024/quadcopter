# VL53L1X 正确使用方法详解

## 一、传感器概览

VL53L1X 是 ST 的二代 ToF（Time-of-Flight，飞行时间）激光测距传感器。它发射 940nm VCSEL 激光脉冲，通过测量光子飞行时间来计算距离。相比前代 VL53L0X，它的主要改进是：

- **最远测距 4m**（VL53L0X 只有 2m）
- **三种距离模式**：Short（~1.3m）、Medium（~3m）、Long（~4m）
- **多区测距**（16 个 ROI 区域）
- **集成直方图算法**，抗环境光能力强

内部有一个 **ARM Cortex-M0 内核的固件** 负责所有信号处理，主机只需要通过 I2C 读写寄存器来控制它。

## 二、硬件连接要点

VL53L1X 是纯粹的 I2C 从设备，硬件连接很简单但有几个陷阱：

```
STM32 F103            VL53L1X
  PB10 (SCL)  ─────── SCL
  PB11 (SDA)  ─────── SDA
  PB12 (XSHUT) ────── XSHUT
  3.3V        ─────── VIN
  GND         ─────── GND
```

**关键注意点**：

1. **I2C 上拉电阻** — 模块一般自带 4.7kΩ 上拉，如果是裸片则需要外部加上拉
2. **I2C 速率** — 官方支持最高 400kHz（标准 Fast Mode）
3. **3.3V 供电** — VL53L1X **不能接 5V**，会永久损坏
4. **I2C 地址** — 7 位地址 `0x29`（8 位地址 `0x52`），不可更改（除非用 XSHUT 多路复用）
5. **XSHUT** — 这是**硬件复位**引脚，低电平有效。VL53L1X 内部 ROM 引导程序在上电后需要约 1.2ms 完成

## 三、寄存器地址格式

VL53L1X 的寄存器地址是 **16 位大端**格式。I2C 读写时序：

```
START + 设备地址(写) + 寄存器高字节 + 寄存器低字节 + [写入数据 / 重复START+读]
```

**最常见的坑**：寄存器地址是 `uint16_t`，发送时必须先发高 8 位再发低 8 位。

## 四、初始化流程（6 个步骤）

### 步骤 1：硬件复位（XSHUT 序列）

```c
// 1. XSHUT 拉低（复位）
xshut_set(0);
delay_us(100);

// 2. I2C 外设初始化
i2c2_init();

// 3. XSHUT 拉高（释放复位）
xshut_set(1);
delay_us(2000);  // 必须等待 >1.2ms 让内部固件启动
```

VL53L1X 在复位释放后需要时间加载内部 ROM 固件。如果在固件加载完成前就尝试 I2C 通信，会得到 NACK 或错误数据。

### 步骤 2：验证设备身份

```c
uint16_t model_id;
vl53lxx_rd_word(0x010F, &model_id);
// 0x010F-0x0110 寄存器返回 0xEACC
```

VL53L1X：`0xEACC`
VL53L0X：`0xEEAC`

### 步骤 3：等待固件启动完成

```c
// 轮询 0x00E5 寄存器，直到非零
while (boot_state == 0) {
    vl53lxx_boot_state(&boot_state);
    // 超时保护约 1 秒
}
```

寄存器 `FIRMWARE__SYSTEM_STATUS`（0x00E5）的值：**0x03** = 启动完成，**0xFF** = 运行中。

### 步骤 4：下载 ULD 默认配置表

```c
static const uint8_t VL51L1X_DEFAULT_CONFIG[] = { ... 91 bytes ... };
for (addr = 0; addr < 91; addr++) {
    vl53lxx_wr_byte(0x2D + addr, config[addr]);
}
```

这 91 字节覆盖寄存器 0x2D ~ 0x87，包含：
- **时序配置**：0x2D-0x37（sigma 门限、信号门限等）
- **GPIO 配置**：0x38-0x3B（中断极性、输出模式）
- **VHV 配置**：0x3C-0x3F
- **MACROP 配置**：0x50-0x57
- **TCC、Ranging 配置**：0x58-0x7F
- **SPAD 配置**：0x80-0x87

这是 ST ULD（Ultra Lite Driver）的标准配置表，每次上电后必须写入。

### 步骤 5：触发首次测距获取初始化参考

```c
// 启动测距
wr_byte(0x0087, 0x40);
// 等待数据就绪（通过 0x0031 判断，见第八章）
while (!data_ready) { ... }
// 清除中断
wr_byte(0x0086, 0x01);
// 停止测距
wr_byte(0x0087, 0x00);
```

这一步让 VL53L1X 完成一次完整的测距周期，从而校准 SPAD 量程、确定环境光参考水平。

### 步骤 6：配置 VHV 参数

```c
// 配置 VHV 为双边界模式（two bounds VHV）
wr_byte(0x0008, 0x09);
// 从之前温度开始 VHV（不重新校准）
wr_byte(0x000B, 0x00);
```

**说明**：步骤 6 仅配置 VHV（Very High Voltage）相关参数，不涉及距离模式。距离模式（Short/Long）的 VCSEL 周期和 Phasecal 配置是通过 `vl53lxx_set_distance_mode()` 函数单独完成的，详见第五章。

**⚠️ 常见误解纠正**：有些资料提到写寄存器 0x0040-0x0047（VCSEL A/B 周期）和 0x00E9（GROUPED_PARAMETER_HOLD），但经与官方 ULD 驱动对比确认：
- 0x0040-0x0047 **不是** VL53L1X 有效的 VCSEL 周期寄存器（正确的 VCSEL 周期寄存器在 0x0060 和 0x0063）
- 0x00E9 **不是** GROUPED_PARAMETER_HOLD（正确的寄存器是 0x0082）
- 距离模式的正确配置是通过写 **0x004B (PHASECAL_TIMEOUT)**、**0x0060 (VCSEL_PERIOD_A)**、**0x0063 (VCSEL_PERIOD_B)**、**0x0069 (VALID_PHASE_HIGH)**、**0x0078 (WOI_SD0)**、**0x007A (INITIAL_PHASE_SD0)** 来实现的

## 五、距离模式配置

VL53L1X 有三种距离模式：

| 模式 | 最大距离 | 优点 | 缺点 |
|------|---------|------|------|
| **Short** | ~1.3m | 精度高，抗串扰好，支持 15ms 时序预算 | 量程短 |
| **Medium** | ~3m | 均衡模式 | 中等 |
| **Long** | ~4m | 最远量程 | 近距精度稍差，最小时序预算 20ms |

三种模式的区别体现在以下 6 个寄存器的配置上：

| 寄存器 | 地址 | Short 模式值 | Long/Medium 模式值 |
|--------|------|-------------|-------------------|
| PHASECAL_CONFIG__TIMEOUT_MACROP | 0x004B | 0x14 | 0x0A |
| RANGE_CONFIG__VCSEL_PERIOD_A | 0x0060 | 0x07 | 0x0F |
| RANGE_CONFIG__VCSEL_PERIOD_B | 0x0063 | 0x05 | 0x0D |
| RANGE_CONFIG__VALID_PHASE_HIGH | 0x0069 | 0x38 | 0xB8 |
| SD_CONFIG__WOI_SD0 | 0x0078 | 0x0705 | 0x0F0D |
| SD_CONFIG__INITIAL_PHASE_SD0 | 0x007A | 0x0606 | 0x0E0E |

PHASECAL_TIMEOUT（0x004B）是"相位校准超时"——设备内部用这个值来判断校准是否完成。Short 模式下激光脉冲更短，需要更长的校准时间（0x14 = 20），而 Long 模式用 0x0A（10）。

VCSEL_PERIOD_A/B（0x0060/0x0063）是激光脉冲的发射周期，决定了激光脉冲的占空比，直接关联最大可测距离。

**设置模式的正确调用顺序**：

```c
// 1. 先设距离模式（内部会重写 6 个配置寄存器并做相位校准）
vl53lxx_set_distance_mode(VL53LXX_DISTANCEMODE_LONG);

// 2. 再设时序预算（需要知道当前距离模式来选择正确的 lookup table）
vl53lxx_set_timing_budget_ms(50);
```

## 六、时序预算（Timing Budget）

时序预算控制每次测距的执行时间：

- **Short 模式范围**：15ms ~ 500ms
- **Long/Medium 模式范围**：20ms ~ 500ms
- **越短越快**，但精度越差（信号积分时间短）
- **越长越准**，但更新率低

⚠️ **时序预算的寄存器值不是线性公式计算出来的**！**不能**用 `ms * 3750` 或 `ms * 1120` 这类公式。必须使用 ST 官方标定的 lookup table：

```c
// Short 模式 lookup table：
TB=15ms → TIMEOUT_MACROP_A=0x011D, TIMEOUT_MACROP_B=0x0027
TB=20ms → TIMEOUT_MACROP_A=0x0051, TIMEOUT_MACROP_B=0x006E
TB=33ms → TIMEOUT_MACROP_A=0x00D6, TIMEOUT_MACROP_B=0x006E
TB=50ms → TIMEOUT_MACROP_A=0x01AE, TIMEOUT_MACROP_B=0x01E8
TB=100ms→ TIMEOUT_MACROP_A=0x02E1, TIMEOUT_MACROP_B=0x0388
TB=200ms→ TIMEOUT_MACROP_A=0x03E1, TIMEOUT_MACROP_B=0x0496
TB=500ms→ TIMEOUT_MACROP_A=0x0591, TIMEOUT_MACROP_B=0x05C1

// Long/Medium 模式 lookup table：
TB=20ms → TIMEOUT_MACROP_A=0x001E, TIMEOUT_MACROP_B=0x0022
TB=33ms → TIMEOUT_MACROP_A=0x0060, TIMEOUT_MACROP_B=0x006E
TB=50ms → TIMEOUT_MACROP_A=0x00AD, TIMEOUT_MACROP_B=0x00C6
TB=100ms→ TIMEOUT_MACROP_A=0x01CC, TIMEOUT_MACROP_B=0x01EA
TB=200ms→ TIMEOUT_MACROP_A=0x02D9, TIMEOUT_MACROP_B=0x02F8
TB=500ms→ TIMEOUT_MACROP_A=0x048F, TIMEOUT_MACROP_B=0x04A4
```

VL53L1X 有 **A/B 两个测量通道**，时序预算必须同时写入两个通道（0x005E-0x005F 和 0x0061-0x0062），各 2 字节共 4 字节。

在四轴飞行器上，推荐 50ms（即 20Hz 更新率），平衡了响应速度和精度。

## 七、开始测距

```c
// 连续模式 - 传感器自动循环测距
void vl53lxx_start_continuous(void) {
    wr_byte(0x0086, 0x01);     // 清除挂起的中断
    wr_byte(0x0087, 0x40);     // 写入 0x40 启动测距
}
```

写 `0x0087 = 0x40` 后，传感器内部状态机会自动循环：测距 → 产生中断 → 等待清除 → 再次测距。

要停止：
```c
wr_byte(0x0087, 0x00);     // 写入 0x00 停止
```

## 八、读取测量结果

### 判断数据就绪

**推荐方式**：读取 GPIO 中断状态寄存器

```c
uint8_t polarity, status;

// 1. 读 0x0030 获取中断极性（bit 4: 1=低有效, 0=高有效）
vl53lxx_rd_byte(0x0030, &polarity);
polarity = !((polarity & 0x10) >> 4);  // 1=高有效, 0=低有效

// 2. 读 0x0031 的 bit 0，与极性比较
vl53lxx_rd_byte(0x0031, &status);
data_ready = ((status & 0x01) == polarity) ? 1 : 0;
```

`0x0031`（GPIO__TIO_HV_STATUS）的 bit 0 直接映射到 GPIO 中断引脚的逻辑电平。这是 ST 官方 ULD 使用的判断方式，不依赖外部 GPIO 中断。

**不推荐**读 `0x0088`（RESULT_INTERRUPT_STATUS）来判断数据就绪，虽然它也能反映中断状态，但不是 ST 官方推荐的检测方式。

### 读取距离和信号质量

```c
// 距离 (0x0096): uint16 big-endian, 单位 1mm（已做串扰补偿）
// 信号强度 (0x0098): uint16 big-endian, 单位 MCPS（需 *8 转 kcps）
// 环境光 (0x0090): uint16 big-endian, 单位 MCPS（需 *8 转 kcps）
// 测距状态 (0x0089): bit 0-4 编码原始测距状态（需重映射）
```

**结果寄存器 0x0088-0x00B3 共 44 字节包含完整的测距结果块**：

| 偏移 | 长度 | 寄存器地址 | 内容 |
|------|------|-----------|------|
| 0x00 | 1 | 0x0088 | 中断状态（bit 0-2） |
| 0x01 | 1 | 0x0089 | 原始测距状态（bit 0-4） |
| 0x02 | 1 | 0x008A | 已用 SPAD 数 |
| 0x03 | 1 | 0x008B | Stream count |
| 0x04-0x05 | 2 | 0x008C-0x008D | SPAD count |
| 0x06-0x07 | 2 | 0x008E-0x008F | DSS corrected signal rate |
| 0x08-0x09 | 2 | 0x0090-0x0091 | Ambient rate (MCPS) |
| 0x0A-0x0B | 2 | 0x0092-0x0093 | Sigma (mm/256) |
| 0x0C-0x0D | 2 | 0x0094-0x0095 | Phase |
| **0x0E-0x0F** | **2** | **0x0096-0x0097** | **距离 (mm，串扰补偿后)** |

最佳实践是一次性 burst read 读取 44 字节，而不是分多次读。

**注意**：当前 VL53LXX.c 驱动实现中，`vl53lxx_read_result()` 使用独立寄存器读取（非 burst），分别从 0x0089、0x0098、0x0090、0x0096 逐一读取。其中 signal_rate 和 ambient_rate 需要乘以 8 转换为 kcps 单位。

### 测距状态（Range Status）—— 正确解码方式

这是判断数据是否可信的关键。**原始寄存器值是 5 位编码**（`raw & 0x1F`），需要按以下表格重映射：

```c
uint8_t raw_status = read_byte(0x0089) & 0x1F;
uint8_t range_status;

switch (raw_status) {
    case 9:  range_status = 0; break;   // 0 = 数据有效
    case 6:  range_status = 1; break;   // 1 = Sigma 超标
    case 4:  range_status = 2; break;   // 2 = Signal 超标
    case 8:  range_status = 3; break;   // 3 = 环境光过高
    case 5:  range_status = 4; break;   // 4 = Wrap around
    case 3:  range_status = 5; break;   // 5 = 硬件错误
    case 19: range_status = 6; break;   // 6 = 范围未校准
    case 7:  range_status = 7; break;   // 7 = 范围合并
    case 12: range_status = 9; break;   // 9 = 幅度过低
    case 18: range_status = 10; break;  // 10 = Phase 过高
    case 22: range_status = 11; break;  // 11 = 幅度过高
    case 23: range_status = 12; break;  // 12 = 一致性低
    case 13: range_status = 13; break;  // 13 = 范围无效
    default: range_status = 255; break; // 未知状态
}
```

**飞行控制应仅检查是否 range_status == 0。**

**⚠️ 常见错误**：不要用 `(raw_byte >> 5) & 0x07` 来提取状态。虽然老版本 VL53L0X 使用 bit 7-5，但 VL53L1X 的测距状态编码在 bit 0-4，且需要上述重映射。

### 清除中断

读取数据后必须清除中断，否则传感器不会再产生新数据：

```c
wr_byte(0x0086, 0x01);     // 写入 0x01 清除中断
```

清除中断后，传感器在连续模式下会自动开始下一次测距，**不需要**再写 0x0087。

## 九、完整的工作循环

```c
// 初始化
vl53lxx_init();
vl53lxx_set_distance_mode(VL53LXX_DISTANCEMODE_LONG);
vl53lxx_set_timing_budget_ms(50);
vl53lxx_start_continuous();

// 主循环
while (1) {
    vl53lxx_result_t result;
    if (vl53lxx_read_result(&result)) {
        if (result.range_status == 0) {
            // 数据有效，记录距离
            altitude_mm = result.distance_mm;
        } else {
            // 数据无效——可能是信号弱或过近/过远
            // 建议保持上次有效值而不是置零
        }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}
```

### 各数据字段在飞行控制中的用途

| 字段 | 用途 | 说明 |
|------|------|------|
| **distance_mm** | PID 高度控制器直接输入 | 核心测量值，单位 mm，范围 0~4000 |
| **range_status** | 数据有效性校验（关键！） | status==0 才可使用 distance，否则保持上次值 |
| **signal_rate** | 辅助判断地面情况 | 信号过强（>60000）→ 离地太近（<10cm）；信号过弱（<10）→ 飞得太高或地面反射差 |
| **ambient_rate** | 环境干扰检测 | 户外强光下值飙升，此时应降低传感器数据权重 |

对于飞行器高度保持，**最小必需字段**是 `distance_mm` + `range_status`。`signal_rate` 用于增强鲁棒性（检测地面接近饱和）。

## 十、常见问题和排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| I2C 应答错误 | 接线/上拉/速率问题 | 检查上拉电阻，降速到 100kHz 测试 |
| Model ID 不匹配 | 地址读错/芯片不同 | 确认是 VL53L1X 而非 VL53L0X（0xEACC vs 0xEEAC） |
| Boot 超时 | XSHUT 时序不对 | XSHUT 拉低复位后延时 2ms 再拉高 |
| 距离读数偏大 10 倍 | **距离模式配置错误** | 写入 0x0060=0x0F, 0x0063=0x0D, 0x004B=0x0A, 0x0069=0xB8 |
| 距离读数为 0 或闪变 | 中断未清除 | 每次读取后写 0x0086 = 0x01 |
| 近距离不准 | 串扰（光学串扰） | VL53L1X <3-4cm 数据不可信，软件做死区处理 |
| 远距离数据不稳定 | 信号太弱/时序预算短 | 增加时序预算到 100-200ms |
| 时序预算设置无效 | 使用了错误的线性公式 | 必须使用 ST 官方 lookup table，不能计算 |

## 十一、四轴飞行器应用的特殊建议

1. **安装位置**：传感器必须安装在四轴底部，朝下指向地面。确保没有任何物体（脚架、电池线）遮挡光学窗口

2. **光学窗口**：VL53L1X 的发射和接收窗口之间需要物理隔光（通常在模块上有金属隔板或开槽 PCB）。如果自己画 PCB，务必在发射和接收之间留出隔离槽

3. **数据融合**：ToF 传感器的输出有一定噪声（±2-4cm），建议和加速度计/气压计数据做卡尔曼滤波融合

4. **飞控逻辑**：
   - 起飞前先读取一次距离，确认传感器正常工作
   - 高度 <10cm 时禁用 ToF 数据（串扰区，数据不可靠）
   - 检测到 `range_status != 0` 时维持上次高度，不做突变
   - 起飞后高度通过累计变化量平滑过渡

5. **抗地面干扰**：不同地面材质（瓷砖、地毯、草地）反射率不同，会影响信号强度和精度。确保你的应用在所有预期地面材质上测试过

## 附录：VL53L1X 关键寄存器映射

| 地址 | 名称 | 说明 |
|------|------|------|
| 0x0000 | SOFT_RESET | 写 0x00 触发软件复位 |
| 0x0001 | I2C_SLAVE_DEVICE_ADDRESS | I2C 设备地址（7 位） |
| 0x0008 | VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND | VHV 配置（双边界=0x09，全 VHV=0x81） |
| 0x000B | VHV_CONFIG | VHV 起始配置（0=从上次温度开始，0x92=重新校准） |
| 0x0030 | GPIO_HV_MUX_CTRL | GPIO 控制（bit 4: 0=高有效, 1=低有效） |
| **0x0031** | **GPIO_TIO_HV_STATUS** | **GPIO 中断状态（bit 0=中断引脚电平）——用于检测 data ready** |
| 0x004B | PHASECAL_CONFIG_TIMEOUT_MACROP | Phasecal 超时（Short=0x14, Long=0x0A） |
| 0x005E-0x005F | RANGE_CONFIG_TIMEOUT_MACROP_A | 时序预算 A 通道（uint16, big-endian） |
| 0x0060 | RANGE_CONFIG_VCSEL_PERIOD_A | VCSEL A 周期（Short=0x07, Long=0x0F） |
| 0x0061-0x0062 | RANGE_CONFIG_TIMEOUT_MACROP_B | 时序预算 B 通道（uint16, big-endian） |
| 0x0063 | RANGE_CONFIG_VCSEL_PERIOD_B | VCSEL B 周期（Short=0x05, Long=0x0D） |
| 0x0069 | RANGE_CONFIG_VALID_PHASE_HIGH | 有效相位阈值（Short=0x38, Long=0xB8） |
| 0x0078-0x0079 | SD_CONFIG_WOI_SD0 | WOI 配置（Short=0x0705, Long=0x0F0D） |
| 0x007A-0x007B | SD_CONFIG_INITIAL_PHASE_SD0 | 初始相位（Short=0x0606, Long=0x0E0E） |
| 0x0082 | SYSTEM_GROUPED_PARAMETER_HOLD | 组参数保持（注意：不是 0x00E9！） |
| 0x0086 | SYSTEM_INTERRUPT_CLEAR | 写 0x01 清除中断 |
| 0x0087 | SYSTEM_MODE_START | 0x40=启动测距, 0x00=停止 |
| 0x0088 | RESULT_INTERRUPT_STATUS | 中断状态 |
| **0x0089** | **RESULT_RANGE_STATUS** | **原始测距状态（bit 0-4，需重映射）** |
| 0x0090-0x0091 | RESULT_AMBIENT_COUNT_RATE_MCPS_SD | 环境光噪声（单位 MCPS，*8=kcps） |
| **0x0096-0x0097** | **RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0** | **最终距离（uint16, big-endian, 单位 mm，串扰补偿后）** |
| 0x0098-0x0099 | RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 | 峰值信号率（单位 MCPS，*8=kcps） |
| 0x00E5 | FIRMWARE_SYSTEM_STATUS | 固件状态（非零=启动完成） |
| 0x010F-0x0110 | IDENTIFICATION_MODEL_ID | 芯片标识（0xEACC = VL53L1X） |
