# 四轴飞行器串级 PID 控制算法详解

## 1. 为什么用串级 PID？

单级 PID 直接对角度做控制：角误差 → PID → 电机 PWM。这种方案响应慢、抗扰动差，
因为实际改变姿态的是**角速度**而非角度本身。

串级 PID 将控制拆分为两层：

```
遥控目标角度 → [角度外环 P] → 目标角速度 → [角速度内环 PID] → 电机 PWM
                    ↑                              ↑
              当前角度(惯导)                  当前角速度(陀螺仪)
```

内环响应频率远高于外环，能快速抑制阵风等高频扰动，是业界飞控的标准架构。

---

## 2. 系统参数总览

### 2.1 硬件平台

| 参数 | 值 |
|------|-----|
| MCU | STM32F103C8T6 @ 72MHz |
| IMU | MPU6050，陀螺仪 ±1000°/s，加速度计 ±8g |
| 姿态解算 | Mahony AHRS，200Hz 更新 |
| 电机 | 8520 空心杯 × 4，独立 PWM 电调 |
| PWM | TIM2，50Hz（20ms 周期），脉宽 1000-2300μs |
| 控制频率 | 200Hz（5ms 周期） |

### 2.2 控制参数

```c
// 角度外环（纯 P）
#define PID_ANGLE_KP        10.0f       // 角度误差 → 目标角速度 (rad/s / rad)

// 角速度内环
#define PID_RATE_ROLL_KP    30.0f       // Roll  P
#define PID_RATE_ROLL_KI    5.0f        // Roll  I
#define PID_RATE_ROLL_KD    2.0f        // Roll  D
#define PID_RATE_PITCH_KP   30.0f       // Pitch P
#define PID_RATE_PITCH_KI   5.0f        // Pitch I
#define PID_RATE_PITCH_KD   2.0f        // Pitch D
#define PID_RATE_YAW_KP     0.0f        // Yaw   P（调试期关闭）
#define PID_RATE_YAW_KI     0.0f        // Yaw   I（调试期关闭）
#define PID_RATE_YAW_KD     0.0f        // Yaw   D（调试期关闭）

// 限幅
#define PID_RATE_INTEGRAL_LIMIT   200.0f   // 积分限幅 (μs)
#define PID_RATE_OUTPUT_LIMIT     300.0f   // 输出限幅 (μs)
#define PID_ANGLE_RATE_LIMIT      8.0f     // 角速度限幅 (rad/s ≈ 458°/s)
#define PID_MAX_ANGLE             0.7854f  // 最大倾斜角 (rad ≈ 45°)
```

---

## 3. 数据流详解

### 3.1 完整控制链路

```
┌─────────────────────────────────────────────────────────────────┐
│                        vTaskFCDebug (200Hz)                      │
│                                                                  │
│  MPU6050 ─→ gyro(°/s)→DEG2RAD→(gx,gy,gz)                       │
│           ─→ acce(m/s²)────────→(ax,ay,az)                      │
│                  ↓                                               │
│           mahony_update()  ─→  euler(roll,pitch,yaw) 单位: rad  │
│                                                                  │
│  NRF24 ──→ g_rc_data[HU_RX]=0~255  ─→  rc_roll                  │
│         ──→ g_rc_data[HU_RY]=0~255  ─→  rc_pitch                │
│         ──→ g_rc_data[HU_LX]=0~255  ─→  rc_yaw                  │
│         ──→ g_rc_data[HU_LY]=0~255  ─→  rc_throttle             │
│                  ↓                                               │
│           cascade_pid_update(roll, pitch,                        │
│                              gyro_x, gyro_y, gyro_z,            │
│                              rc_roll, rc_pitch, rc_yaw,         │
│                              rc_throttle, armed, dt)            │
│                  ↓                                               │
│           TIM2 CH1~CH4 ─→ M8520 空心杯电机                       │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 遥控器映射表（HU-M40 协议）

| 通道 | 摇杆 | 物理动作 | 原始值 | 死区 | 映射目标 | 范围 |
|------|------|---------|--------|------|---------|------|
| HU_RX (4) | 右横滚 | 左/右 | 0..127..255 | ±10 | 目标 roll 角 | ±45° (±0.785 rad) |
| HU_RY (5) | 右俯仰 | 上/下 | 0..127..255 | ±10 | 目标 pitch 角 | ±45° (±0.785 rad) |
| HU_LX (6) | 左偏航 | 左/右 | 0..127..255 | ±10 | 目标 yaw 角速率 | ±8 rad/s (±458°/s) |
| HU_LY (7) | 左油门 | 下/上 | 255..127..0 | — | 基准油门 PWM | 2300..1650..1000 μs |

### 3.3 RC 死区处理

```c
static float rc_deadband(int32_t raw, int32_t center)
{
    int32_t diff = raw - center;
    if (diff > -RC_DEADBAND && diff < RC_DEADBAND) {  // ±10 范围
        return 0.0f;  // 死区内输出 0，消除摇杆中位抖动
    }
    return (float)diff / RC_HALF_RANGE;  // 归一化到 [-1, 1]
}
```

---

## 4. 角度外环（P 控制器）

### 4.1 算法

```
angle_error = target_angle - current_angle     // 单位: rad
target_rate = angle_kp × angle_error           // 单位: rad/s
target_rate = clamp(target_rate, ±PID_ANGLE_RATE_LIMIT)
```

### 4.2 代码对应

```c
// cascade_PID.c L159-L174
angle_error_roll  = g_target_roll  - roll;       // 角度误差
angle_error_pitch = g_target_pitch - pitch;

target_rate_roll  = g_angle_kp * angle_error_roll;   // P × 误差 = 目标角速度
target_rate_pitch = g_angle_kp * angle_error_pitch;

// 角速度限幅，防止过激机动
if (target_rate_roll  > PID_ANGLE_RATE_LIMIT)  target_rate_roll  =  PID_ANGLE_RATE_LIMIT;
if (target_rate_roll  < -PID_ANGLE_RATE_LIMIT) target_rate_roll  = -PID_ANGLE_RATE_LIMIT;
// ... pitch 同理
```

### 4.3 参数物理意义

`PID_ANGLE_KP = 10.0` 的含义：

| 角度误差 | 输出目标角速度 | 换算 |
|---------|-------------|------|
| 1° (0.0175 rad) | 0.175 rad/s | ≈ 10°/s |
| 10° (0.175 rad) | 1.75 rad/s | ≈ 100°/s |
| 45° (0.785 rad) | 7.85 rad/s | ≈ 450°/s（达到限幅） |

**为什么只用 P？** 角度外环不需要积分——角速度内环的 I 项已经能消除稳态误差；
不需要微分——内环直接用陀螺仪数据做 D。

---

## 5. 角速度内环（PID 控制器）

### 5.1 完整 PID 公式

```
rate_error = target_rate - gyro_raw              // 单位: rad/s

integral += rate_error × dt                      // 积分累加
integral = clamp(integral, ±integral_limit)       // 积分限幅

derivative = (rate_error - prev_error) / dt       // 微分（误差差分）

output = kp × rate_error
       + ki × integral
       + kd × derivative

output = clamp(output, ±output_limit)             // 输出限幅
```

### 5.2 代码实现

```c
// cascade_PID.c L39-L69
float pid_update(pid_t *pid, float error, float dt)
{
    float output, derivative;

    if (dt < 1e-6f) dt = 0.0025f;  // 防除零，默认 400Hz

    // I: 积分累加 + 限幅（防积分饱和 windup）
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    // D: 误差微分（不是"测量值微分"）
    derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // PID 合成
    output = pid->kp * error
           + pid->ki * pid->integral
           + pid->kd * derivative;

    // 输出限幅
    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
```

### 5.3 参数物理意义

内环 PID 的输出单位是 **PWM 脉宽偏移量（μs）**。

| 参数 | Roll/Pitch | 含义 |
|------|-----------|------|
| kp = 30 | 1 rad/s 误差 → 30μs 修正 | 比例：即时响应 |
| ki = 5 | 1 rad/s·s 累积 → 5μs 修正 | 积分：消除稳态偏置 |
| kd = 2 | 1 rad/s² 变化 → 2μs 修正 | 微分：抑制超调振荡 |

**限幅含义**：
- 积分上限 ±200μs → 最大稳态修正量
- 输出上限 ±300μs → 单次最大 PWM 偏移（距基准油门）

### 5.4 三种轴的控制差异

| 轴 | 外环 | 内环 | 原因 |
|----|------|------|------|
| Roll | 角度→角速度(P) | 角速度→PWM(PID) | 有重力参考，可做角度闭环 |
| Pitch | 角度→角速度(P) | 角速度→PWM(PID) | 有重力参考，可做角度闭环 |
| Yaw | —（无外环） | RC→角速度(PID) | **无磁力计，Mahony yaw 会漂移，无法做角度闭环** |

Yaw 走纯速率控制：遥控器偏航杆直接给出目标角速率，陀螺仪 Z 轴做反馈。
这意味飞手需要手动维持偏航方向（类似传统直升机尾舵）。

---

## 6. 电机混控（Mixer）

### 6.1 X 构型电机布局

```
            前方
        M3(FL,CCW)   M1(FR,CW)
             \         /
              +-------+
             /         \
        M2(RL,CW)    M4(RR,CCW)
            后方

    CH1 = PA0 = M1 = 前右 (Front-Right, CW)
    CH2 = PA1 = M2 = 后左 (Rear-Left,  CW)
    CH3 = PA2 = M3 = 前左 (Front-Left,  CCW)
    CH4 = PA3 = M4 = 后右 (Rear-Right, CCW)
```

### 6.2 混控公式

```
M1 = throttle + roll + pitch - yaw
M2 = throttle - roll - pitch - yaw
M3 = throttle - roll + pitch + yaw
M4 = throttle + roll - pitch + yaw
```

### 6.3 混控逻辑验证

以 **pitch 抬头纠正**（pid_pitch > 0，需要 nose-up 力矩）为例：

| 电机 | 位置 | pitch 贡献 | 效果 |
|------|------|-----------|------|
| M1 | 前右 | +pitch → 加速 ↑↑ | ✓ 前加速 → 抬头 |
| M3 | 前左 | +pitch → 加速 ↑↑ | ✓ 前加速 → 抬头 |
| M2 | 后左 | -pitch → 减速 ↓↓ | ✓ 后减速 → 抬头 |
| M4 | 后右 | -pitch → 减速 ↓↓ | ✓ 后减速 → 抬头 |

以 **roll 右倾纠正**（pid_roll > 0，需要 right-up 力矩）为例：

| 电机 | 位置 | roll 贡献 | 效果 |
|------|------|----------|------|
| M1 | 前右 | +roll → 加速 ↑↑ | ✓ 右加速 → 右倾 |
| M4 | 后右 | +roll → 加速 ↑↑ | ✓ 右加速 → 右倾 |
| M2 | 后左 | -roll → 减速 ↓↓ | ✓ 左减速 → 右倾 |
| M3 | 前左 | -roll → 减速 ↓↓ | ✓ 左减速 → 右倾 |

以 **yaw 右旋纠正**（pid_yaw > 0，需要 CW 力矩）为例：

| 电机 | 旋向 | yaw 贡献 | 效果 |
|------|------|---------|------|
| M3 | CCW | +yaw → 加速 ↑↑ | ✓ CCW 加速 → CW 扭矩 |
| M4 | CCW | +yaw → 加速 ↑↑ | ✓ CCW 加速 → CW 扭矩 |
| M1 | CW | -yaw → 减速 ↓↓ | ✓ CW 减速 → CW 扭矩 |
| M2 | CW | -yaw → 减速 ↓↓ | ✓ CW 减速 → CW 扭矩 |

### 6.4 PWM 钳位

```c
// 每个电机输出限制在 1000~2300μs，防止电调异常
if (m1 < 1000) m1 = 1000;  if (m1 > 2300) m1 = 2300;
// ... m2, m3, m4 同理
```

---

## 7. 安全保护机制

### 7.1 层级保护

```
┌──────────────────────────────────────────────┐
│ 第 1 层: 默认 Disarm                          │
│   上电后 g_fc_armed=0，所有电机=1000μs        │
├──────────────────────────────────────────────┤
│ 第 2 层: 按键解锁/上锁                        │
│   K1 按下 → armed=1, PID复位                  │
│   K2 按下 → armed=0                           │
├──────────────────────────────────────────────┤
│ 第 3 层: 失控保护 (Failsafe)                   │
│   RC 信号丢失 > 500ms → armed=0               │
├──────────────────────────────────────────────┤
│ 第 4 层: 油门怠速切断                         │
│   throttle < 1100μs → PID输出清零 + 积分复位  │
└──────────────────────────────────────────────┘
```

### 7.2 PID 边沿复位

```c
// 仅在 armed→disarmed 状态变化时复位一次，不是每次循环
if (!armed) {
    if (s_was_armed) {
        pid_reset(&g_pid_rate_roll);
        pid_reset(&g_pid_rate_pitch);
        pid_reset(&g_pid_rate_yaw);
        s_was_armed = 0;
    }
    // 设电机最低，直接返回
    return;
}
s_was_armed = 1;
```

### 7.3 油门－PID 解耦

当油门低于怠速阈值时，**主动清零 PID 输出并复位积分**：

```c
if (g_base_throttle < MOTOR_IDLE_PWM) {  // 1100μs
    pid_roll = pid_pitch = pid_yaw = 0.0f;
    pid_reset(&g_pid_rate_roll);   // 防止恢复油门时积分冲击
    pid_reset(&g_pid_rate_pitch);
    pid_reset(&g_pid_rate_yaw);
}
```

---

## 8. 控制策略总结

### 8.1 整体架构

```
                 ┌─ 角度外环 (P, 200Hz) ─┐
  RC 摇杆 ──→ 角度误差 ──→ 目标角速度 ──┤
                 └───────────────────────┘
                                            ┌─ 角速度内环 (PID, 200Hz) ─┐
                    陀螺仪 ──→ 角速度误差 ──→ PID ──→ 混控 ──→ 4路PWM ──┤
                                            └───────────────────────────┘
                                                      ↑
                                         加速度计 ──→ Mahony ──→ 角度
```

### 8.2 调节优先级

飞行器调试时的 PID 调参顺序：

```
第 1 步: 关 Yaw (kp=ki=kd=0)，只调 Roll/Pitch
          ↓
第 2 步: 先调内环 P（增大到微振荡，取 60%~70%）
          ↓
第 3 步: 加内环 D（抑制 P 引起的超调，直到手感"干净"）
          ↓
第 4 步: 加内环 I（消除稳态误差，限幅从小开始）
          ↓
第 5 步: 调外环 P（增大到目标角度响应满意）
          ↓
第 6 步: 最后开启 Yaw 内环（用较小增益，接受手控修正）
```

### 8.3 当前调试状态

| 状态 | Roll/Pitch | Yaw |
|------|-----------|-----|
| 外环 P | ✅ 启用 (kp=10) | —（无外环） |
| 内环 PID | ✅ 启用 | ❌ 关闭（kp=ki=kd=0） |
| 原因 | 有重力参考，角度闭环可行 | Mahony yaw 漂移，无磁力计修正 |

### 8.4 已知限制

1. **Yaw 漂移**：Mahony 算法无磁力计，yaw 角随时间漂移。当前通过关闭 yaw 内环避免误纠正。
   改进方向：加磁力计（HMC5883L/QMC5883L）实现 yaw 角度闭环。

2. **地面测试积分饱和**：飞行器放在桌面无法改变姿态，PID 误差永远不为零，积分会饱和到上限。
   **这是正常现象，起飞后会自动消散。** 调试时可通过油门拉低（< 1100μs）来主动复位积分。

3. **无高度控制**：VL53L1X 激光测距模块驱动已就绪，但高度环 PID 未实现（后续扩展）。

4. **参数需飞行实测**：当前 PID 增益为保守默认值，需要在实际飞行中根据响应特性微调。

---

## 9. 接口速查

### 9.1 初始化

```c
cascade_pid_init();  // 在飞控任务初始化阶段调用一次
```

### 9.2 主循环调用

```c
cascade_pid_update(
    euler.roll,     euler.pitch,      // 当前姿态角 (rad)
    gyro_x,         gyro_y,  gyro_z,  // 当前角速度 (rad/s)
    rc_roll,        rc_pitch, rc_yaw, rc_throttle,  // RC 通道 (0-255)
    g_fc_armed,                        // 解锁状态
    dt                                 // 采样周期 (s)
);
```

### 9.3 调试监控

```c
// 全局变量，可被调试任务读取
extern float   g_target_roll;      // 目标 roll 角 (rad)
extern float   g_target_pitch;     // 目标 pitch 角 (rad)
extern float   g_target_yaw_rate;  // 目标 yaw 角速率 (rad/s)
extern uint16_t g_base_throttle;   // 基准油门 (μs)
extern uint8_t  g_fc_armed;        // 解锁标志
extern uint16_t g_motor_output[4]; // 四路电机 PWM (μs)
```
