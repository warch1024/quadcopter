#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

typedef struct {
    float q0, q1, q2, q3;
} mahony_quat_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} mahony_euler_t;

void mahony_init(float kp, float ki, float halfT);
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float halfT);
void mahony_get_euler(mahony_euler_t *euler);
void mahony_get_quat(mahony_quat_t *quat);

void vTaskMPU6050Test(void *pvParameters);
void vTaskMahonyTest(void *pvParameters);

#endif
