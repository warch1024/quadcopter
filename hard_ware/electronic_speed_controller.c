#include "electronic_speed_controller.h"
#include "M8520.h"
#include "communication.h"
#include "VL53LXX.h"
#include "cascade_PID.h"
#include "attitude.h"
#include "MPU6050.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "string.h"
#include "uart.h"



void vTaskESCControlTest(void *pvParameters)
{
    uint8_t ly;
    int16_t throttle;
    uint8_t dat[16];
    uint8_t connected = 0;

    NRF24L01_Init();
    NRF24L01_RX_Mode(g_tx_addr, g_tx_addr);
    m8520_init();
    m8520_unlock();
    printf("ESC initialized, CH1=PA0 CH2=PA1 CH3=PA2 CH4=PA3\n");
    printf("NRF24L01 init done, waiting for HU-M40...\n");

    while (1) {
        if (hu_m40_receive_frame(dat)) {
            if (dat[0] == 1 && dat[1] == 3 && dat[15] == hu_m40_checksum(dat, 15)) {
                if (!connected) {
                    connected = 1;
                    g_rc_connected = 1;
                    printf(">>> HU-M40 CONNECTED <<<\n");
                }

                memcpy(g_rc_data, dat, 16);

                hu_m40_send_ack(dat);
                NRF24L01_RX_Mode(g_rx_addr, g_rx_addr);
                memcpy(g_tx_addr, g_rx_addr, 5);

                ly = g_rc_data[HU_LY];
                throttle = M8520_PULSE_MIN + (int16_t)(255 - ly) * (M8520_PULSE_MAX - M8520_PULSE_MIN) / 255;

                m8520_set_throttle(M8520_CH1, (uint16_t)throttle);
                m8520_set_throttle(M8520_CH2, (uint16_t)throttle);
                m8520_set_throttle(M8520_CH3, (uint16_t)throttle);
                m8520_set_throttle(M8520_CH4, (uint16_t)throttle);

                printf("LY=%3d THR=%4d\n", ly, throttle);
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
// ***************上层应用—— 测试VL53L1X*************** */
void vTaskVL53LXXDebugTest(void *pvParameters)
{
    uint8_t ret;
    vl53lxx_result_t result;
    uint32_t sample_count = 0;
    uint32_t invalid_count = 0;
    uint8_t sensor_ready = 0;
    uint32_t loop_count = 0;

    printf("\n=== VL53L1X Test ===\n");

    ret = vl53lxx_init();
    if (ret == VL53LXX_OK) {
        vl53lxx_set_distance_mode(VL53LXX_DISTANCEMODE_LONG);
        vl53lxx_set_timing_budget_ms(50);
        sensor_ready = 1;
        printf("VL53L1X ready\n");
    } else {
        printf("VL53L1X init fail (ret=%d)\n", ret);
    }

    if (sensor_ready) {
        vl53lxx_start_continuous();
    }

    while (1) {
        if (!sensor_ready) {    //重试初始化
            if (loop_count % 50 == 0) {
                ret = vl53lxx_init();
                if (ret == VL53LXX_OK) {
                    vl53lxx_set_distance_mode(VL53LXX_DISTANCEMODE_LONG);
                    vl53lxx_set_timing_budget_ms(20);
                    vl53lxx_start_continuous();
                    sensor_ready = 1;
                    printf("VL53L1X retry ready\n");
                } else {
                    printf("VL53L1X retry init fail (ret=%d)\n", ret);
                }
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }
        if (vl53lxx_read_result(&result)) {
            sample_count++;

            if (result.range_status != 0 || result.distance_mm > 4000) {
                    printf("  invalid #%lu dist=%d st=%d sig=%d\n",
                           invalid_count, result.distance_mm, result.range_status, result.signal_rate);
            } else {
                printf("[%6lu] %5dmm  sig=%5d  st=%d\n",
                       sample_count, result.distance_mm, result.signal_rate, result.range_status);
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
        loop_count++;
        // vl53lxx_dump_results();
    }
}

void vTaskFCDebug(void *pvParameters)
{
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mahony_euler_t euler;
    float gx, gy_r, gz;
    TickType_t last_tick, now_tick, t_i2c_start, t_i2c_end, t_pid_end;
    float real_dt;
    uint16_t loop_cnt = 0;
    fc_state_t last_state = FC_STATE_WAIT_RC;
    rc_input_t rc;

    /* ── 计时诊断变量 ── */
    uint32_t loop_ticks, max_loop_ticks = 0;
    uint32_t i2c_ticks,    max_i2c_ticks = 0;
    uint32_t pid_ticks,    max_pid_ticks = 0;
    uint32_t i2c_err,      max_i2c_err   = 0;  // 窗口内 I2C 错误数
    uint32_t slow_loops = 0;
    uint32_t last_i2c_total = 0;               // 上窗口 I2C 累计错误基准

    /* ── 初始化 ── */
    if (mpu6050_init() != MPU6050_OK) {
        printf("MPU6050 init failed!\n");
        vTaskDelete(NULL);
        return;
    }
    printf("MPU6050 init OK, calibrating gyro...\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    mpu6050_calibrate_gyro();
    printf("Gyro calibration done\n");

    m8520_init();
    m8520_unlock();
    printf("ESC init done\n");

    last_tick = xTaskGetTickCount();
    mahony_init(2.0f, 0.005f, 0.0025f);
    cascade_pid_init();

    /* 初始化 RC 数据为空（等待 RC 任务推送） */
    memset(&rc, 0, sizeof(rc));

    printf("\n=== FC Task ===\n");
    printf("State: WAIT_RC->IDLE->(K1)ARMED  K2=disarm\n");
    printf("[WAIT_RC] waiting for RC task...\n");

    while (1) {
        /* ── 1. 传感器读取 ── */
        t_i2c_start = xTaskGetTickCount();
        mpu6050_get_acce(&acce);
        mpu6050_get_gyro(&gyro);
        t_i2c_end = xTaskGetTickCount();

        /* I2C 错误追踪：这次 I2C 读有没有新增错误 */
        {
            uint32_t curr = mpu6050_get_i2c_total_errors();
            if (curr > last_i2c_total) {
                max_i2c_err += (curr - last_i2c_total);
                last_i2c_total = curr;
            }
        }

        now_tick = xTaskGetTickCount();

        /* ── 计时：循环原始耗时（在 last_tick 更新前记录） ── */
        loop_ticks = now_tick - last_tick;
        i2c_ticks  = t_i2c_end - t_i2c_start;
        if (loop_ticks > max_loop_ticks) max_loop_ticks = loop_ticks;
        if (i2c_ticks  > max_i2c_ticks)  max_i2c_ticks  = i2c_ticks;
        if (loop_ticks > 15) slow_loops++;   // tick>15 即单循环>15ms 视为异常慢

        real_dt = (float)loop_ticks * portTICK_PERIOD_MS / 1000.0f;
        last_tick = now_tick;

        if (real_dt > 0.2f) {
            real_dt = 0.005f;           // 上限保护（启动后首帧保护）
        } else if (real_dt < 0.001f) {
            real_dt = 0.01f;            // 下限保护（对应 ~100Hz 控制频率，匹配传感器刷新）
        }

        gx  = gyro.gyro_x * 0.01745329252f;
        gy_r = gyro.gyro_y * 0.01745329252f;
        gz  = gyro.gyro_z * 0.01745329252f;

        /* ── 2. 姿态解算 ── */
        mahony_update(gx, gy_r, gz, acce.acce_x, acce.acce_y, acce.acce_z, real_dt * 0.5f);
        mahony_get_euler(&euler);

        /* ── 3. 从消息队列获取最新遥控数据（非阻塞） ── */
        if (g_rc_queue != NULL) {
            rc_input_t new_rc;
            if (xQueueReceive(g_rc_queue, &new_rc, 0) == pdPASS) {
                rc = new_rc;
            }
        }

        /* ── 4. 级联 PID 控制 ── */
        cascade_pid_update(euler.roll, euler.pitch,
                           gx, gy_r, gz,
                           rc.stick_roll, rc.stick_pitch,
                           rc.stick_yaw, rc.stick_throttle,
                           rc.rc_alive,
                           (rc.buttons & HU_K1) ? 1 : 0,
                           (rc.buttons & HU_K2) ? 1 : 0,
                           real_dt);
        t_pid_end = xTaskGetTickCount();
        pid_ticks = t_pid_end - t_i2c_end;   // 从 I2C 结束到 PID 结束
        if (pid_ticks > max_pid_ticks) max_pid_ticks = pid_ticks;

        /* ── 5. 状态变化提示 ── */
        if (g_fc_state != last_state) {
            switch (g_fc_state) {
            case FC_STATE_WAIT_RC:
                printf("[WAIT_RC] RC lost, motors off\n");
                break;
            case FC_STATE_IDLE:
                printf("[IDLE] RC connected, throttle=1150 — press K1 to arm\n");
                break;
            case FC_STATE_ARMED:
                printf("\n[ARMED] === flight control active ===\n");
                printf("%-6s %-6s %-7s %-6s %-5s "
                       "%-5s %-5s %-5s %-5s %-3s "
                       "%-5s %-5s %-5s %-5s %-4s %-4s %-4s\n",
                       "ROLL", "PITCH", "YAW", "GZ", "AZ",
                       "T_RL", "T_PT", "T_YW", "THR", "ST",
                       "M1", "M2", "M3", "M4",
                       "DTms", "I2C", "ERR");
                break;
            case FC_STATE_FAILSAFE:
                printf("[FAILSAFE] RC lost in flight — auto-level hold\n");
                break;
            }
            last_state = g_fc_state;
        }

        /* ── 6. 定期串口输出 ── */
        if (g_fc_state == FC_STATE_WAIT_RC) {
            if (loop_cnt % 100 == 0) {
                printf("[WAIT_RC] roll=%+.1f pitch=%+.1f dt=%lums\n",
                       euler.roll * 57.29578f,
                       euler.pitch * 57.29578f,
                       max_loop_ticks);
                max_loop_ticks = 0;
                max_i2c_ticks  = 0;
                max_i2c_err    = 0;
                slow_loops = 0;
            }
        } else if (g_fc_state == FC_STATE_IDLE) {
            if (loop_cnt % 100 == 0) {
                printf("[IDLE] roll=%+d.%d pitch=%+d.%d  THR=%u  dtmax=%lums i2c=%lu err=%lu slow=%lu\n",
                       (int)euler.roll * 57, (int)(euler.roll * 572.9578f) % 10,
                       (int)euler.pitch * 57, (int)(euler.pitch * 572.9578f) % 10,
                       g_base_throttle,
                       max_loop_ticks, max_i2c_ticks, max_i2c_err, slow_loops);
                max_loop_ticks = 0;
                max_i2c_ticks  = 0;
                max_i2c_err    = 0;
                slow_loops = 0;
            }
        } else {
            if (loop_cnt % 40 == 0) {
                const char *state_str =
                    (g_fc_state == FC_STATE_ARMED)    ? "ARM" :
                    (g_fc_state == FC_STATE_FAILSAFE) ? "FS"  : "??";

                /* 末尾3列：DTms(窗口内最差循环tick)、I2C(I2C耗时tick)、ERR(I2C错误数) */
                printf("%+6.1f %+6.1f %+6.1f %+5.1f %+5.1f "
                       "%+5.1f %+5.1f %+5.1f %4u %-3s "
                       "%4u %4u %4u %4u "
                       "%4lu %4lu %4lu\n",
                       euler.roll  * 57.29578f,
                       euler.pitch * 57.29578f,
                       euler.yaw   * 57.29578f,
                       gyro.gyro_z,
                       acce.acce_z,
                       g_target_roll  * 57.29578f,
                       g_target_pitch * 57.29578f,
                       g_target_yaw_rate * 57.29578f,
                       g_base_throttle,
                       state_str,
                       g_motor_output[0], g_motor_output[1],
                       g_motor_output[2], g_motor_output[3],
                       max_loop_ticks, max_i2c_ticks, max_i2c_err);
                max_loop_ticks = 0;
                max_i2c_ticks  = 0;
                max_i2c_err    = 0;
                slow_loops = 0;
            }
        }

        loop_cnt++;
        vTaskDelay(5 / portTICK_PERIOD_MS);  // 5ms → 200Hz，降低控制延迟
    }
}



