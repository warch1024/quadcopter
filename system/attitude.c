#include "attitude.h"
#include "MPU6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "math.h"

#define DEG_TO_RAD  0.01745329252f
#define RAD_TO_DEG  57.2957795131f

static float g_q0 = 1.0f, g_q1 = 0.0f, g_q2 = 0.0f, g_q3 = 0.0f;    // 角速度四元数
static float g_ix = 0.0f, g_iy = 0.0f;                                // 角速度积分项
static float g_kp = 0.0f, g_ki = 0.0f;                                // 比例项和积分项

void mahony_init(float kp, float ki, float halfT)
{
    g_q0 = 1.0f;
    g_q1 = 0.0f;
    g_q2 = 0.0f;
    g_q3 = 0.0f;
    g_ix = 0.0f;
    g_iy = 0.0f;
    g_kp = kp;
    g_ki = ki;
    (void)halfT;
}
/**
 * @brief   Mahony姿态解算算法更新函数
 * @param   gx,gy,gz  陀螺仪三轴原始角速度(rad/s)
 * @param   ax,ay,az  加速度计三轴原始加速度值
 * @param   halfT     采样周期的一半 (dt/2)，用于四元数积分
 * @note    全局变量说明：
 *          g_q0~g_q3 : 姿态四元数 q0 q1 q2 q3
 *          g_kp      : 比例系数
 *          g_ki      : 积分系数
 *          g_ix/g_iy : 积分误差累计项
 */
//加速度计归一化 → 计算理论重力向量 → 求姿态误差 → PI 补偿陀螺仪 → 四元数更新 + 归一化
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float halfT)
{
    float norm;         // 向量模长
    float vx, vy, vz;   // 由当前四元数推算出的理论重力向量
    float ex, ey, ez;   // 加速度计实测重力与理论重力的误差向量
    float qa, qb, qc;   // 临时变量，保存更新前的四元数，防止迭代覆盖

    // 1. 加速度计数据归一化：将加速度向量转为单位向量
    norm = sqrtf(ax * ax + ay * ay + az * az);  // 计算加速度向量模长
    // 模长接近0，判定数据异常，直接退出本次解算
    if (norm < 1e-6f) {
        return;
    }
    norm = 1.0f / norm;
    ax *= norm;  // X轴归一化
    ay *= norm;  // Y轴归一化
    az *= norm;  // Z轴归一化
    // 2. 根据当前姿态四元数，计算机体坐标系下的理论重力向量(单位向量)
    vx = 2.0f * (g_q1 * g_q3 - g_q0 * g_q2);
    vy = 2.0f * (g_q0 * g_q1 + g_q2 * g_q3);
    vz = g_q0 * g_q0 - g_q1 * g_q1 - g_q2 * g_q2 + g_q3 * g_q3;
    // 3. 计算误差向量：实测重力向量（陀螺仪测量的） 与 理论重力向量 的叉乘误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    // 4. 积分环节：积分限幅修正陀螺仪角速度，抑制静态漂移
    if (g_ki > 0.0f) {
        g_ix += g_ki * ex * halfT * 2.0f;  // X轴误差积分累加
        g_iy += g_ki * ey * halfT * 2.0f;  // Y轴误差积分累加
        gx += g_ix;  // 积分项补偿陀螺仪X轴
        gy += g_iy;  // 积分项补偿陀螺仪Y轴
    }
    // 5. 比例环节：比例项修正陀螺仪角速度，快速修正姿态偏差
    gx += g_kp * ex;
    gy += g_kp * ey;
    gz += g_kp * ez;
    // 6. 保存更新前的四元数，避免计算过程中数值被覆盖
    qa = g_q0;
    qb = g_q1;
    qc = g_q2;
    // 7. 四元数微分方程积分，更新姿态四元数
    g_q0 += (-qb * gx - qc * gy - g_q3 * gz) * halfT;
    g_q1 += ( qa * gx + qc * gz - g_q3 * gy) * halfT;
    g_q2 += ( qa * gy - qb * gz + g_q3 * gx) * halfT;
    g_q3 += ( qa * gz + qb * gy - qc * gx) * halfT;
    // 8. 四元数归一化：保证四元数始终为单位四元数，消除累计误差
    norm = sqrtf(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);
    norm = 1.0f / norm;
    g_q0 *= norm;
    g_q1 *= norm;
    g_q2 *= norm;
    g_q3 *= norm;
}
/* **********i2c1_功能封装--获取校准后的欧拉角****************** */
void mahony_get_euler(mahony_euler_t *euler)
{
    if (euler == NULL) {
        return;
    }
    euler->roll  = atan2f(2.0f * (g_q0 * g_q1 + g_q2 * g_q3),
                          1.0f - 2.0f * (g_q1 * g_q1 + g_q2 * g_q2));
    euler->pitch = asinf(2.0f * (g_q0 * g_q2 - g_q3 * g_q1));
    euler->yaw   = atan2f(2.0f * (g_q0 * g_q3 + g_q1 * g_q2),
                          1.0f - 2.0f * (g_q2 * g_q2 + g_q3 * g_q3));
}
/* **********i2c1_功能封装--获取校准后的四元数****************** */
void mahony_get_quat(mahony_quat_t *quat)
{
    if (quat == NULL) {
        return;
    }
    quat->q0 = g_q0;
    quat->q1 = g_q1;
    quat->q2 = g_q2;
    quat->q3 = g_q3;
}

void vTaskMPU6050Test(void *pvParameters)
{
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    float temp;
    uint8_t ret;

    ret = mpu6050_init();
    if (ret != MPU6050_OK) {
        printf("MPU6050 init failed! err=%d\n", ret);
        vTaskDelete(NULL);
        return;
    }
    printf("MPU6050 init OK, calibrating gyro...\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    mpu6050_calibrate_gyro();
    printf("Gyro calibration done\n\n");

    while (1) {
        mpu6050_get_acce(&acce);
        mpu6050_get_gyro(&gyro);
        temp = mpu6050_get_temp();

        printf("%+7.6f,%+7.6f,%+7.6f,%+7.6f,%+7.6f,%+7.6f,%5.1f\n",
               acce.acce_x, acce.acce_y, acce.acce_z,
               gyro.gyro_x, gyro.gyro_y, gyro.gyro_z, temp);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

#if 1   //测试Mahony算法
void vTaskMahonyTest(void *pvParameters)
{
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mahony_euler_t euler;
    float gx, gy, gz;
    TickType_t last_tick, now_tick;
    float real_dt;
    uint8_t ret;

    ret = mpu6050_init();
    if (ret != MPU6050_OK) {
        printf("MPU6050 init failed! err=%d\n", ret);
        vTaskDelete(NULL);
        return;
    }
    printf("MPU6050 init OK, calibrating gyro...\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    mpu6050_calibrate_gyro();
    printf("Gyro calibration done\n");

    last_tick = xTaskGetTickCount();
    mahony_init(2.0f, 0.005f, 0.0025f);

    while (1) {
        mpu6050_get_acce(&acce);    //m/s^2
        mpu6050_get_gyro(&gyro);    //度/s

        now_tick = xTaskGetTickCount();
        real_dt = (float)(now_tick - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        last_tick = now_tick;

        if (real_dt > 0.2f) {
            real_dt = 0.005f;
        }

        gx = gyro.gyro_x * DEG_TO_RAD;
        gy = gyro.gyro_y * DEG_TO_RAD;
        gz = gyro.gyro_z * DEG_TO_RAD;

        mahony_update(gx, gy, gz, acce.acce_x, acce.acce_y, acce.acce_z, real_dt * 0.5f);
        mahony_get_euler(&euler);

        printf("%+7.6f,%+7.6f,%+7.6f\n",
               euler.roll * RAD_TO_DEG, euler.pitch * RAD_TO_DEG, euler.yaw * RAD_TO_DEG);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
#endif
