#include "cascade_PID.h"
#include "M8520.h"
#include "FreeRTOS.h"
#include "task.h"

pid_t g_pid_rate_roll;  // 横滚角速率 PID
pid_t g_pid_rate_pitch; // 俯仰角速率 PID
pid_t g_pid_rate_yaw;   // 偏航角速率 PID
float g_angle_kp = PID_ANGLE_KP;     // 角度 PID 比例系数

float g_target_roll;  // 目标横滚角速率
float g_target_pitch; // 目标俯仰角速率
float g_target_yaw_rate; // 目标偏航角速率
uint16_t g_base_throttle; // 基础油门值

uint8_t  g_fc_armed = 0; // 是否解锁
uint8_t  g_fc_failsafe = 0; // 是否进入失控自稳模式
fc_state_t g_fc_state = FC_STATE_WAIT_RC; // 状态机状态
uint16_t g_motor_output[4]; // 电机输出值

static uint8_t s_was_armed = 0;  // 上一次是否解锁
static float   s_failsafe_throttle = 0.0f;  // 失控自稳悬停的油门值
static float   s_failsafe_elapsed  = 0.0f;  // 失控自稳保持时间 (ms)
static uint8_t s_k1_prev = 0;   // 上一次的 K1 状态

#define RC_DEADBAND           1 // RC 死区范围
#define RC_CENTER             127 // RC 中心值
#define RC_HALF_RANGE         127.0f // RC 半范围
#define RC_THROTTLE_MIN       10 // RC 最小油门值
/**
 * @brief 初始化 PID 控制器
 * 
 * @param pid PID 控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param integral_limit 积分限幅
 * @param output_limit 输出限幅
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}
/**
 * @brief 更新 PID 控制器输出
 * 
 * @param pid PID 控制器指针
 * @param error 错误值
 * @param dt 时间间隔 (秒)
 * @return float 输出值
 */
float pid_update(pid_t *pid, float error, float dt)
{
    float output;
    float derivative;
    float p_term, i_term, d_term;

    if (dt < 1e-6f) {   //防止 dt 过小（除零保护）
        dt = 0.0025f;   //0.0025s（400Hz）
    }

    //微分项计算（阻尼、抗震荡、稳姿态）
    derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    //比例项（响应当前误差）
    p_term = pid->kp * error;
    //微分项（抗震荡、稳姿态）
    d_term = pid->kd * derivative;
    //积分项（消除静差、稳悬停）
    i_term = pid->ki * pid->integral;

    //PID 最终输出公式——先算不包含新积分的输出，用于反算饱和
    output = p_term + i_term + d_term;

    //输出限幅（保护电机、防止超调）+ 抗积分饱和（conditional integration）
    if (output > pid->output_limit) {
        output = pid->output_limit;
        // 输出已饱和上限，仅当误差会减少积分时才累加
        if (error < 0.0f) {
            pid->integral += error * dt;
        }
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
        // 输出已饱和下限，仅当误差会减少积分时才累加
        if (error > 0.0f) {
            pid->integral += error * dt;
        }
    } else {
        // 未饱和，正常累加积分
        pid->integral += error * dt;
    }

    //积分限幅（二次保护）
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    return output;
}
/**
 * @brief 重置 PID 控制器
 * 
 * @param pid PID 控制器指针
 */
void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
/**
 * @brief 初始化级联 PID 控制器
 * 
 */
void cascade_pid_init(void)
{
    pid_init(&g_pid_rate_roll,  PID_RATE_ROLL_KP,  PID_RATE_ROLL_KI,  PID_RATE_ROLL_KD,
             PID_RATE_INTEGRAL_LIMIT, PID_RATE_OUTPUT_LIMIT);
    pid_init(&g_pid_rate_pitch, PID_RATE_PITCH_KP, PID_RATE_PITCH_KI, PID_RATE_PITCH_KD,
             PID_RATE_INTEGRAL_LIMIT, PID_RATE_OUTPUT_LIMIT);
    pid_init(&g_pid_rate_yaw,   PID_RATE_YAW_KP,   PID_RATE_YAW_KI,   PID_RATE_YAW_KD,
             PID_RATE_INTEGRAL_LIMIT, PID_RATE_OUTPUT_LIMIT);

    g_angle_kp = PID_ANGLE_KP;

    g_target_roll = 0.0f;
    g_target_pitch = 0.0f;
    g_target_yaw_rate = 0.0f;
    g_base_throttle = M8520_PULSE_MIN;

    g_fc_armed = 0;
    g_fc_failsafe = 0;
    g_fc_state = FC_STATE_WAIT_RC;

    g_motor_output[0] = M8520_PULSE_MIN;
    g_motor_output[1] = M8520_PULSE_MIN;
    g_motor_output[2] = M8520_PULSE_MIN;
    g_motor_output[3] = M8520_PULSE_MIN;

    cascade_pid_motor_output(
        M8520_PULSE_MIN, M8520_PULSE_MIN,
        M8520_PULSE_MIN, M8520_PULSE_MIN);
}
/**
 * @brief 应用 RC 死区
 * 
 * @param raw RC 原始值
 * @param center RC 中心值
 * @return float 应用死区后的值
 */
static float rc_deadband(int32_t raw, int32_t center)
{
    int32_t diff = raw - center;
    if (diff > -RC_DEADBAND && diff < RC_DEADBAND) {
        return 0.0f;
    }
    return (float)diff / RC_HALF_RANGE;
}

/**
 * @brief 映射油门值
 * 
 * @param rc_src RC 原始油门值
 * @return uint16_t 映射后的油门值
 */
static uint16_t map_throttle(float rc_src)
{
    /* 两段映射：上半段 0~127→1150~2300，下半段 127~255→1000~1150 */
    if (rc_src < RC_THROTTLE_MIN) rc_src = 0.0f;
    if (rc_src <= 127.0f) {
        /* 摇杆上半段：0(满油)→2300, 127(中位)→1150 */
        return 1150
             + (uint16_t)((127.0f - rc_src) * (float)(M8520_PULSE_MAX - 1150) / 127.0f);
    } else {
        /* 摇杆下半段：127(中位)→1150, 255(最低)→1000 */
        return M8520_PULSE_MIN
             + (uint16_t)((255.0f - rc_src) * 150.0f / 128.0f);
    }
}
/**
 * @brief 更新级联 PID 控制器输出
 * 
 * @param roll 俯仰角
 * @param pitch 横滚角
 * @param gyro_x 陀螺仪 X 轴
 * @param gyro_y 陀螺仪 Y 轴
 * @param gyro_z 陀螺仪 Z 轴
 * @param rc_roll 俯仰角 RC 原始值
 * @param rc_pitch 横滚角 RC 原始值
 * @param rc_yaw 偏航角 RC 原始值
 * @param rc_throttle_source 油门 RC 原始值
 * @param rc_alive RC 信号是否有效
 * @param k1 K1 按键状态
 * @param k2 K2 按键状态
 * @param dt 时间间隔 (秒)
 */
void cascade_pid_update(float roll, float pitch,
                        float gyro_x, float gyro_y, float gyro_z,
                        float rc_roll, float rc_pitch, float rc_yaw,
                        float rc_throttle_source,
                        uint8_t rc_alive, uint8_t k1, uint8_t k2,
                        float dt)
{
    float angle_error_roll, angle_error_pitch;
    float target_rate_roll, target_rate_pitch;
    float rate_error_roll, rate_error_pitch, rate_error_yaw;
    float pid_roll, pid_pitch, pid_yaw;
    float throttle;
    float m1, m2, m3, m4;

    /* ══════════════════════════════════════════
     *  状态迁移（rc_alive 由任务层 tick 判定，已含超时逻辑）
     * ══════════════════════════════════════════ */
    switch (g_fc_state) {

    case FC_STATE_WAIT_RC:
        if (rc_alive) {
            g_fc_state = FC_STATE_IDLE;
        }
        break;

    case FC_STATE_IDLE:
        if (!rc_alive) {
            g_fc_state = FC_STATE_WAIT_RC;
        } else if (k1 && !s_k1_prev) {
            g_fc_state = FC_STATE_ARMED;
            pid_reset(&g_pid_rate_roll);
            pid_reset(&g_pid_rate_pitch);
            pid_reset(&g_pid_rate_yaw);
        }
        break;

    case FC_STATE_ARMED:
        if (!rc_alive) {
            g_fc_state = FC_STATE_FAILSAFE;
            s_failsafe_throttle = (float)g_base_throttle;
            s_failsafe_elapsed  = 0.0f;
            pid_reset(&g_pid_rate_roll);
            pid_reset(&g_pid_rate_pitch);
            pid_reset(&g_pid_rate_yaw);
        } else if (k2 && !s_k1_prev) {
            g_fc_state = FC_STATE_IDLE;
            pid_reset(&g_pid_rate_roll);
            pid_reset(&g_pid_rate_pitch);
            pid_reset(&g_pid_rate_yaw);
        }
        break;

    case FC_STATE_FAILSAFE:
        if (rc_alive) {
            g_fc_state = FC_STATE_IDLE;
            pid_reset(&g_pid_rate_roll);
            pid_reset(&g_pid_rate_pitch);
            pid_reset(&g_pid_rate_yaw);
        } else {
            s_failsafe_elapsed += dt * 1000.0f;
            if (s_failsafe_elapsed > FC_FAILSAFE_HOLD_MS) {
                g_fc_state = FC_STATE_WAIT_RC;
                pid_reset(&g_pid_rate_roll);
                pid_reset(&g_pid_rate_pitch);
                pid_reset(&g_pid_rate_yaw);
            }
        }
        break;
    }
    s_k1_prev = k1;

    /* ══════════════════════════════════════════
     *  各状态的控制行为
     * ══════════════════════════════════════════ */

    /* ── WAIT_RC: 全停 ── */
    if (g_fc_state == FC_STATE_WAIT_RC) {
        g_motor_output[0] = M8520_PULSE_MIN;
        g_motor_output[1] = M8520_PULSE_MIN;
        g_motor_output[2] = M8520_PULSE_MIN;
        g_motor_output[3] = M8520_PULSE_MIN;
        cascade_pid_motor_output(
            M8520_PULSE_MIN, M8520_PULSE_MIN,
            M8520_PULSE_MIN, M8520_PULSE_MIN);
        g_target_roll  = 0.0f;
        g_target_pitch = 0.0f;
        g_target_yaw_rate = 0.0f;
        g_base_throttle = M8520_PULSE_MIN;
        return;
    }

    /* ── IDLE: 怠速 1150，无姿态控制 ── */
    if (g_fc_state == FC_STATE_IDLE) {
        g_base_throttle = 1150;
        g_target_roll  = 0.0f;
        g_target_pitch = 0.0f;
        g_target_yaw_rate = 0.0f;
        g_motor_output[0] = 1150;
        g_motor_output[1] = 1150;
        g_motor_output[2] = 1150;
        g_motor_output[3] = 1150;
        cascade_pid_motor_output(1150, 1150, 1150, 1150);
        return;
    }

    /* ── FAILSAFE: 自稳 + 油门缓降 ── */
    if (g_fc_state == FC_STATE_FAILSAFE) {
        /* 自动回平 */
        g_target_roll  = 0.0f;
        g_target_pitch = 0.0f;
        g_target_yaw_rate = 0.0f;

        /* 油门：前 500ms 保持，之后线性降到 1150 */
        if (s_failsafe_elapsed < 500.0f) {
            throttle = s_failsafe_throttle;
        } else {
            float ramp = (s_failsafe_elapsed - 500.0f) / (FC_FAILSAFE_HOLD_MS - 500.0f);
            if (ramp > 1.0f) ramp = 1.0f;
            throttle = s_failsafe_throttle
                     - ramp * (s_failsafe_throttle - 1150.0f);
        }
        g_base_throttle = (uint16_t)throttle;
    }
    /* ── ARMED: 正常飞控 ── */
    else {
        g_target_roll  = rc_deadband((int32_t)rc_roll,  RC_CENTER) * PID_MAX_ANGLE;
        g_target_pitch = rc_deadband((int32_t)rc_pitch, RC_CENTER) * PID_MAX_ANGLE;
        g_target_yaw_rate = rc_deadband((int32_t)rc_yaw, RC_CENTER) * PID_ANGLE_RATE_LIMIT;
        g_base_throttle = map_throttle(rc_throttle_source);
    }

    /* ══════════════════════════════════════════
     *  PID 解算 (ARMED / FAILSAFE 共用)
     * ══════════════════════════════════════════ */
    // 计算角度误差
    angle_error_roll  = g_target_roll  - roll;  //目标横滚角  - 当前横滚角
    angle_error_pitch = g_target_pitch - pitch; //目标俯仰角  - 当前俯仰角
    //角度环 P 计算 → 输出目标角速度
    target_rate_roll  = g_angle_kp * angle_error_roll;  // 角度P参数 × 角度误差
    target_rate_pitch = g_angle_kp * angle_error_pitch; // 角度P参数 × 角度误差
    //👆这是外环（角度环），只有 P 控制，无 I/D 控制
    // 限制输出范围
    if (target_rate_roll > PID_ANGLE_RATE_LIMIT) {
        target_rate_roll = PID_ANGLE_RATE_LIMIT;
    } else if (target_rate_roll < -PID_ANGLE_RATE_LIMIT) {
        target_rate_roll = -PID_ANGLE_RATE_LIMIT;
    }
    if (target_rate_pitch > PID_ANGLE_RATE_LIMIT) {
        target_rate_pitch = PID_ANGLE_RATE_LIMIT;
    } else if (target_rate_pitch < -PID_ANGLE_RATE_LIMIT) {
        target_rate_pitch = -PID_ANGLE_RATE_LIMIT;
    }
    //4.计算角速度误差
    rate_error_roll  = target_rate_roll  - gyro_x; //目标横滚角速度  - 当前横滚角速度
    rate_error_pitch = target_rate_pitch - gyro_y; //目标俯仰角速度  - 当前俯仰角速度
    rate_error_yaw   = g_target_yaw_rate - gyro_z; //目标偏航角速度  - 当前偏航角速度
    //5.计算输出pid 内环（速率环），完整 PID 控制
    pid_roll  = pid_update(&g_pid_rate_roll,  rate_error_roll,  dt); //横滚角速度误差  - 目标横滚角速度
    pid_pitch = pid_update(&g_pid_rate_pitch, rate_error_pitch, dt); //俯仰角速度误差  - 目标俯仰角速度
    pid_yaw   = pid_update(&g_pid_rate_yaw,   rate_error_yaw,   dt); //偏航角速度误差  - 目标偏航角速度

    /* 油门怠速时关闭 PID */
    if (g_base_throttle < MOTOR_IDLE_PWM) {
        pid_roll  = 0.0f;
        pid_pitch = 0.0f;
        pid_yaw   = 0.0f;
        pid_reset(&g_pid_rate_roll);
        pid_reset(&g_pid_rate_pitch);
        pid_reset(&g_pid_rate_yaw);
    }

    throttle = (float)g_base_throttle;  // 转换为浮点数

    /*  + 字型 (Plus) 混控
     *        M3(前,CCW,PA2)
     *            |
     *  M2(左,CW,PA1)--+--M1(右,CW,PA0)
     *            |
     *        M4(后,CCW,PA3)
     */
    //6.四轴动力分配逻辑，把 PID 结果变成 4 个电机转速：
    m1 = throttle + pid_roll - pid_yaw;        // 右: +roll, CW→-yaw逆时针
    m2 = throttle - pid_roll - pid_yaw;        // 左: -roll, CW→-yaw逆时针
    m3 = throttle + pid_pitch + pid_yaw;       // 前: +pitch, CCW→+yaw逆时针
    m4 = throttle - pid_pitch + pid_yaw;       // 后: -pitch, CCW→+yaw逆时针
    //7. PWM 限幅 + 输出电机转速
    if (m1 < M8520_PULSE_MIN) m1 = M8520_PULSE_MIN;
    if (m1 > M8520_PULSE_MAX) m1 = M8520_PULSE_MAX;
    if (m2 < M8520_PULSE_MIN) m2 = M8520_PULSE_MIN;
    if (m2 > M8520_PULSE_MAX) m2 = M8520_PULSE_MAX;
    if (m3 < M8520_PULSE_MIN) m3 = M8520_PULSE_MIN;
    if (m3 > M8520_PULSE_MAX) m3 = M8520_PULSE_MAX;
    if (m4 < M8520_PULSE_MIN) m4 = M8520_PULSE_MIN;
    if (m4 > M8520_PULSE_MAX) m4 = M8520_PULSE_MAX;

    g_motor_output[0] = (uint16_t)m1;
    g_motor_output[1] = (uint16_t)m2;
    g_motor_output[2] = (uint16_t)m3;
    g_motor_output[3] = (uint16_t)m4;

    cascade_pid_motor_output(
        (uint16_t)m1, (uint16_t)m2,
        (uint16_t)m3, (uint16_t)m4);
}

void cascade_pid_motor_output(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    m8520_set_throttle(M8520_CH1, m1);
    m8520_set_throttle(M8520_CH2, m2);
    m8520_set_throttle(M8520_CH3, m3);
    m8520_set_throttle(M8520_CH4, m4);
}
