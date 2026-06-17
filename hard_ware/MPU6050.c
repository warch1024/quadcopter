#include "MPU6050.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stdio.h"

#define I2C1_SCL_PIN    GPIO_Pin_6
#define I2C1_SDA_PIN    GPIO_Pin_7
#define I2C1_GPIO       GPIOB

static float g_acce_sensitivity;
static float g_gyro_sensitivity;
static float g_gyro_bias_x = 0.0f;
static float g_gyro_bias_y = 0.0f;
static float g_gyro_bias_z = 0.0f;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static uint32_t g_i2c_error_count = 0;        // 连续 I2C 错误计数
static uint32_t g_i2c_reset_count  = 0;        // 总线复位次数
static uint32_t g_i2c_total_errors = 0;        // 累计错误总数（只增不减）

#define I2C_LOCK()   do { if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY); } while(0)
#define I2C_UNLOCK() do { if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex); } while(0)

static void i2c1_bus_reset(void);

/* I2C 错误时：累计计数，连续3次则复位总线 */
static void i2c1_on_error(void)
{
    g_i2c_error_count++;
    g_i2c_total_errors++;
    if (g_i2c_error_count >= 3) {
        i2c1_bus_reset();
        g_i2c_reset_count++;
        g_i2c_error_count = 0;
    }
}

static void delay_us(uint32_t us)
{
    volatile uint32_t cnt = us * 18;
    while (cnt--) {
        __NOP();
    }
}
/* **********i2c1_功能封装--初始化I2C1****************** */
static void i2c1_init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    gpio.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C1_GPIO, &gpio);

    I2C_DeInit(I2C1);
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}
/* **********i2c1_功能封装--重置I2C1****************** */
static void i2c1_bus_reset(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    I2C_DeInit(I2C1);

    gpio.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C1_GPIO, &gpio);

    GPIO_SetBits(I2C1_GPIO, I2C1_SDA_PIN | I2C1_SCL_PIN);
    delay_us(10);

    for (i = 0; i < 9; i++) {
        GPIO_ResetBits(I2C1_GPIO, I2C1_SCL_PIN);
        delay_us(10);
        GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN);
        delay_us(10);
    }

    GPIO_ResetBits(I2C1_GPIO, I2C1_SCL_PIN);
    delay_us(10);
    GPIO_ResetBits(I2C1_GPIO, I2C1_SDA_PIN);
    delay_us(10);
    GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN);
    delay_us(10);
    GPIO_SetBits(I2C1_GPIO, I2C1_SDA_PIN);
    delay_us(10);

    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C1_GPIO, &gpio);

    i2c1_init();
}
/* **********i2c1_功能封装--写入寄存器****************** */
static uint8_t i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint32_t timeout;
    uint8_t ret = MPU6050_OK;

    I2C_LOCK();

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }
    (void)I2C1->SR2;

    I2C_SendData(I2C1, reg);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    I2C_SendData(I2C1, data);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);

done:
    I2C_UNLOCK();
    if (ret != MPU6050_OK) {
        i2c1_on_error();
    } else {
        g_i2c_error_count = 0;  /* 成功则清零错误计数 */
    }
    return ret;
}
/* **********i2c1_功能封装--读取寄存器****************** */
static uint8_t i2c1_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t timeout;
    uint8_t ret = MPU6050_OK;

    I2C_LOCK();

    if (len == 0) {
        goto done;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }
    (void)I2C1->SR2;

    I2C_SendData(I2C1, reg);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Receiver);
    timeout = MPU6050_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)) {
        if (--timeout == 0) {
            ret = MPU6050_ERR_I2C;
            goto done;
        }
    }

    if (len == 1) {
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        (void)I2C1->SR2;
        I2C_GenerateSTOP(I2C1, ENABLE);

        timeout = MPU6050_I2C_TIMEOUT;
        while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE)) {
            if (--timeout == 0) {
                ret = MPU6050_ERR_I2C;
                goto done;
            }
        }
        *buf = I2C_ReceiveData(I2C1);
    } else {
        (void)I2C1->SR2;

        while (len) {
            if (len == 1) {
                I2C_AcknowledgeConfig(I2C1, DISABLE);
            }
            timeout = MPU6050_I2C_TIMEOUT;
            while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE)) {
                if (--timeout == 0) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    I2C_AcknowledgeConfig(I2C1, ENABLE);
                    ret = MPU6050_ERR_I2C;
                    goto done;
                }
            }
            *buf++ = I2C_ReceiveData(I2C1);
            len--;
        }

        I2C_GenerateSTOP(I2C1, ENABLE);
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);

done:
    I2C_UNLOCK();
    if (ret != MPU6050_OK) {
        i2c1_on_error();
    } else {
        g_i2c_error_count = 0;  /* 成功则清零错误计数 */
    }
    return ret;
}
/* **********i2c1_功能封装--校准陀螺仪****************** */
void mpu6050_calibrate_gyro(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    mpu6050_gyro_value_t gyro;
    int i;

    g_gyro_bias_x = 0.0f;
    g_gyro_bias_y = 0.0f;
    g_gyro_bias_z = 0.0f;

    printf("  gyro cal: ");
    for (i = 0; i < 500; i++) {
        mpu6050_get_gyro(&gyro);
        sum_x += gyro.gyro_x;
        sum_y += gyro.gyro_y;
        sum_z += gyro.gyro_z;
        if (i % 40 == 0) printf(".");
        delay_us(1000);
    }

    g_gyro_bias_x = sum_x / 500.0f;
    g_gyro_bias_y = sum_y / 500.0f;
    g_gyro_bias_z = sum_z / 500.0f;
    printf(" done (bias: %.3f, %.3f, %.3f)\n",
           g_gyro_bias_x, g_gyro_bias_y, g_gyro_bias_z);
}

/* **********i2c1_功能封装--初始化MPU6050****************** */
uint8_t mpu6050_init(void)
{
    uint8_t whoami;
    uint8_t ret;

    i2c1_init();

    if (g_i2c_mutex == NULL) {
        g_i2c_mutex = xSemaphoreCreateMutex();
    }

    ret = i2c1_read_bytes(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &whoami, 1);
    if (ret != MPU6050_OK) {
        return MPU6050_ERR_I2C;
    }
    if (whoami != 0x68) {
        return MPU6050_ERR_ID;
    }

    i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00);
    i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_SMPRT_DIV, 0x00);
    i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_CONFIG, MPU6050_DLPF_CFG_94HZ);    // 低通94Hz，降低传感器滞后
    i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_RANGE_1000);
    i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_8G);

    g_gyro_sensitivity = MPU6050_GYRO_1000_COEFFICIENT; // 1000 dps ±1000°/s
    g_acce_sensitivity = MPU6050_ACCEL_8G_COEFFICIENT; // 8g ±8g

    return MPU6050_OK;
}
/* **********i2c1_功能封装--获取经量程转换后的原始加速度****************** */
void mpu6050_get_acce(mpu6050_acce_value_t *acce)
{
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    if (acce == NULL) {
        return;
    }

    i2c1_read_bytes(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf, 6);

    raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    acce->acce_x = (float)raw_x / g_acce_sensitivity;
    acce->acce_y = (float)raw_y / g_acce_sensitivity;
    acce->acce_z = (float)raw_z / g_acce_sensitivity;
}
/* **********i2c1_功能封装--获取经量程转换和校准偏移后的原始陀螺仪****************** */
void mpu6050_get_gyro(mpu6050_gyro_value_t *gyro)
{
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    if (gyro == NULL) {
        return;
    }

    i2c1_read_bytes(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H, buf, 6);

    raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    gyro->gyro_x = (float)raw_x / g_gyro_sensitivity - g_gyro_bias_x;
    gyro->gyro_y = (float)raw_y / g_gyro_sensitivity - g_gyro_bias_y;
    gyro->gyro_z = (float)raw_z / g_gyro_sensitivity - g_gyro_bias_z;
}
/* **********i2c1_功能封装--获取温度****************** */
float mpu6050_get_temp(void)
{
    uint8_t buf[2];
    int16_t raw;

    i2c1_read_bytes(MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H, buf, 2);
    raw = (int16_t)((buf[0] << 8) | buf[1]);

    return 36.53f + (float)raw / 340.0f;
}
/* **********I2C 诊断接口****************** */
uint32_t mpu6050_get_i2c_errors(void)
{
    return g_i2c_error_count;
}

uint32_t mpu6050_get_i2c_resets(void)
{
    return g_i2c_reset_count;
}

uint32_t mpu6050_get_i2c_total_errors(void)
{
    return g_i2c_total_errors;
}
