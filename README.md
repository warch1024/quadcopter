# Quadcopter Flight Controller

基于 STM32F103C8T6 + FreeRTOS 的四轴飞行器飞控固件，支持 NRF24L01 无线遥控、MPU6050 姿态解算、级联 PID 控制。

## 硬件平台

| 组件 | 型号 |
|------|------|
| MCU | STM32F103C8T6 (Cortex-M3, 72MHz) |
| IMU | MPU6050 (6 轴陀螺仪 + 加速度计) |
| 无线 | NRF24L01+ 2.4GHz SPI |
| 电调 | M8520 空心杯 + PWM 驱动 |
| 遥控 | HU-M40 遥控器 (NRF24L01 兼容) |
| 测距 | VL53L1X ToF 激光测距 (可选) |

## 软件架构

```
quadcopter/
├── user/               # main.c — 系统初始化、FreeRTOS 任务创建
├── system/             # 核心算法
│   ├── cascade_PID     级联 PID 控制器 (角度外环 + 速率内环)
│   ├── attitude         Mahony 互补滤波姿态解算
│   └── communication    NRF24L01 + HU-M40 遥控协议
├── hard_ware/          # 硬件驱动
│   ├── MPU6050         I2C 传感器驱动 (含自动恢复)
│   ├── NRF24L01        SPI 无线模块驱动
│   ├── M8520           PWM 电机驱动
│   ├── uart            非阻塞串口打印
│   └── electronic_speed_controller  主飞控任务
├── freeRTOS/           FreeRTOS v10 内核源码
└── RTE/                Keil CMSIS 启动文件
```

## FreeRTOS 任务架构

```
优先级 6: vTaskRCReceiver    (5ms)    NRF24L01 遥控接收 + 数据队列推送
优先级 5: vTaskFCDebug       (5ms)    IMU 读取 → 姿态解算 → PID → 电机输出
优先级 4: task_usart1_print  (异步)   非阻塞串口打印缓存
优先级 3: task_usart1_recv   (异步)   串口接收
优先级 1: vTask1             (1s)     LED 心跳
```

## 控制算法

### 级联 PID

```
RC摇杆 → 角度外环(P) → 目标角速度 → 速率内环(PID) → 混控器 → 4路PWM
                ↑                            ↑
           当前姿态角                    当前陀螺仪角速度
```

| 参数 | 自由飞 | 说明 |
|------|--------|------|
| Angle KP | 10.0 | 角度外环 P (角度→角速度) |
| Rate KP | 30.0 | 速率内环 P |
| Rate KI | 5.0 | 速率内环 I (消除静差) |
| Rate KD | 2.0 | 速率内环 D (阻尼抗震荡) |
| Output Limit | ±300 | 电机修正最大幅度 (PWM 单位) |
| Integral Limit | ±60 | 积分限幅 (防饱和) |

### 混控器 (+ 字型 Plus)

```
        M3(前,CCW)
            |
M2(左,CW)---+---M1(右,CW)
            |
        M4(后,CCW)
```

### 姿态解算

Mahony 互补滤波 (PI 型)，融合陀螺仪积分与加速度计观测。

### DLPF 滤波器

MPU6050 内部低通滤波器设为 94Hz (DLPF_CFG=2)，降低传感器延迟。

## 飞控状态机

```
WAIT_RC ──RC连接──→ IDLE ──K1解锁──→ ARMED ──RC断连──→ FAILSAFE
   ↑                  ↑    ←──K2上锁──    │                        │
   └──────────── RC断连超时 ──────────────┘                        │
   └──────────── RC恢复 ───────────────────────────────────────────┘
```

## 编译 & 烧录

- IDE: Keil MDK-ARM 5
- 目标: `quad_copter.uvprojx`
- 编译输出: `Objects/quad_copter.hex`
- 烧录工具: ST-Link / J-Link

## 遥控协议 (HU-M40)

- 无线: NRF24L01 2.4GHz, 16 字节数据帧
- 地址: `HXFB0` (默认)
- 通道: RX(roll), RY(pitch), LX(yaw), LY(throttle)
- 按键: K1(解锁), K2(上锁) 等 6 键
- 数据速率: 200Hz 遥控接收

## 串口诊断输出 (200ms/行)

```
 ROLL PITCH   YAW    GZ    AZ T_RL T_PT T_YW  THR ST   M1   M2   M3   M4 DTms I2C ERR
+1.2  -0.5  +0.3  +2.1 +1.0 +0.0 +0.0 +0.0 1150 ARM 1150 1150 1150 1150    5   1   0
```

## 特性

- I2C 总线自动恢复: 连续 3 次通信失败后执行 bit-bang 复位
- 非阻塞串口打印: 浮点输出不阻塞飞控任务
- 失调保护 (Failsafe): RC 断连后自动回平 + 油门缓降
- 抗积分饱和: conditional integration + 积分限幅
- 控制频率: 200Hz (5ms 间隔，DLPF 94Hz)

## License

MIT
