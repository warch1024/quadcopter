#ifndef __CASCADE_PID_H__
#define __CASCADE_PID_H__

#include "stm32f10x.h"

#define PID_AXIS_ROLL    0
#define PID_AXIS_PITCH   1
#define PID_AXIS_YAW     2

/* ── 原始自由飞参数（无线飞行） ── */
#define PID_RATE_ROLL_KP    30.0f
#define PID_RATE_ROLL_KI    5.0f
#define PID_RATE_ROLL_KD    2.0f

#define PID_RATE_PITCH_KP   30.0f
#define PID_RATE_PITCH_KI   5.0f
#define PID_RATE_PITCH_KD   2.0f

#define PID_RATE_YAW_KP     20.0f
#define PID_RATE_YAW_KI     3.0f
#define PID_RATE_YAW_KD     1.0f

#define PID_ANGLE_KP        10.0f

#define PID_RATE_INTEGRAL_LIMIT   60.0f
#define PID_RATE_OUTPUT_LIMIT     300.0f
#define PID_ANGLE_RATE_LIMIT      6.0f

#define PID_MAX_ANGLE       0.65f   /* ~37° */

#define MOTOR_IDLE_PWM      1100

/* 飞控状态机 */
typedef enum {
    FC_STATE_WAIT_RC  = 0,  // 等待遥控器连接，电机全停
    FC_STATE_IDLE     = 1,  // 遥控已连接，怠速1150，等待解锁
    FC_STATE_ARMED    = 2,  // 已解锁，正常飞行控制
    FC_STATE_FAILSAFE = 3,  // 失控保护，自稳悬停→降落
} fc_state_t;

#define FC_RC_TIMEOUT_MS       1500    // RC 超时判定 (ms)
#define FC_FAILSAFE_HOLD_MS    3000    // 失控自稳保持时间 (ms)

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
} pid_t;

extern pid_t g_pid_rate_roll;
extern pid_t g_pid_rate_pitch;
extern pid_t g_pid_rate_yaw;
extern float g_angle_kp;

extern float g_target_roll;
extern float g_target_pitch;
extern float g_target_yaw_rate;
extern uint16_t g_base_throttle;

extern fc_state_t g_fc_state;
extern uint16_t g_motor_output[4];

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);
float pid_update(pid_t *pid, float error, float dt);
void pid_reset(pid_t *pid);

void cascade_pid_init(void);
void cascade_pid_update(float roll, float pitch,
                        float gyro_x, float gyro_y, float gyro_z,
                        float rc_roll, float rc_pitch, float rc_yaw,
                        float rc_throttle_source,
                        uint8_t rc_alive, uint8_t k1, uint8_t k2,
                        float dt);
void cascade_pid_motor_output(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif
