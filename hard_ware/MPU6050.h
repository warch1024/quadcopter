#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f10x.h"

/*MPU6050地址和寄存器地址*/
#define MPU6050_ADDR               0x68

#define MPU6050_REG_WHO_AM_I       0x75
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_SMPRT_DIV      0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_TEMP_OUT_H     0x41
#define MPU6050_REG_GYRO_XOUT_H    0x43

#define MPU6050_ACCEL_RANGE_2G     0x00
#define MPU6050_ACCEL_RANGE_4G     0x08
#define MPU6050_ACCEL_RANGE_8G     0x10
#define MPU6050_ACCEL_RANGE_16G    0x18

#define MPU6050_ACCEL_2G_COEFFICIENT  16384.0f // 2g ±2g 2.67
#define MPU6050_ACCEL_4G_COEFFICIENT  81920.0f // 4g ±4g 3.33
#define MPU6050_ACCEL_8G_COEFFICIENT  4096.0f // 8g ±8g 6.67
#define MPU6050_ACCEL_16G_COEFFICIENT 2048.0f // 16g ±16g 13.33

#define MPU6050_GYRO_RANGE_250     0x00
#define MPU6050_GYRO_RANGE_500     0x08
#define MPU6050_GYRO_RANGE_1000    0x10
#define MPU6050_GYRO_RANGE_2000    0x18

#define MPU6050_GYRO_250_COEFFICIENT 131.0f // 250 dps ±250°/s 
#define MPU6050_GYRO_500_COEFFICIENT 65.5f // 500 dps ±500°/s 
#define MPU6050_GYRO_1000_COEFFICIENT 32.8f // 1000 dps ±1000°/s 
#define MPU6050_GYRO_2000_COEFFICIENT 16.4f // ±2000°/s 

#define MPU6050_DLPF_CFG_260HZ     0x00
#define MPU6050_DLPF_CFG_184HZ     0x01
#define MPU6050_DLPF_CFG_94HZ      0x02
#define MPU6050_DLPF_CFG_44HZ      0x03
#define MPU6050_DLPF_CFG_21HZ      0x04
#define MPU6050_DLPF_CFG_10HZ      0x05
#define MPU6050_DLPF_CFG_5HZ       0x06

#define MPU6050_I2C_TIMEOUT        100000

#define MPU6050_OK                 0x00
#define MPU6050_ERR_I2C            0x01
#define MPU6050_ERR_ID             0x02
#define MPU6050_ERR_INIT           0x03

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_raw_data_t;

typedef struct {
    float acce_x;   //加速度x横滚，＋ == 向右倾斜， - ==左倾
    float acce_y;   //俯仰加速度， + ==向上， - == 向下
    float acce_z;   //z轴加速度， 约为1，重力1g
} mpu6050_acce_value_t;

typedef struct {    //度/s
    float gyro_x;   //横滚，＋ == 向右倾斜， - ==左倾
    float gyro_y;
    float gyro_z;
} mpu6050_gyro_value_t;

uint8_t mpu6050_init(void);
void mpu6050_get_acce(mpu6050_acce_value_t *acce);
void mpu6050_get_gyro(mpu6050_gyro_value_t *gyro);
float mpu6050_get_temp(void);
void mpu6050_calibrate_gyro(void);
uint32_t mpu6050_get_i2c_errors(void);
uint32_t mpu6050_get_i2c_resets(void);
uint32_t mpu6050_get_i2c_total_errors(void);

#endif
