#include "VL53LXX.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stdio.h"

#define I2C2_SCL_PIN    GPIO_Pin_10
#define I2C2_SDA_PIN    GPIO_Pin_11
#define I2C2_GPIO       GPIOB

static const uint8_t VL51L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
    0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00,
};

static SemaphoreHandle_t g_i2c2_mutex = NULL;
static uint8_t g_distance_mode = VL53LXX_DISTANCEMODE_LONG;

#define I2C_LOCK()   do { if (g_i2c2_mutex) xSemaphoreTake(g_i2c2_mutex, portMAX_DELAY); } while(0)
#define I2C_UNLOCK() do { if (g_i2c2_mutex) xSemaphoreGive(g_i2c2_mutex); } while(0)

static void delay_us(uint32_t us)
{
    volatile uint32_t cnt = us * 18;
    while (cnt--) {
        __NOP();
    }
}
/* **********硬件相关初始化********** */
static void xshut_set(uint8_t level)
{
    if (level) {
        GPIO_SetBits(VL53LXX_XSHUT_GPIO, VL53LXX_XSHUT_PIN);
    } else {
        GPIO_ResetBits(VL53LXX_XSHUT_GPIO, VL53LXX_XSHUT_PIN);
    }
}

static void xshut_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = VL53LXX_XSHUT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(VL53LXX_XSHUT_GPIO, &gpio);

    xshut_set(0);
}

static void i2c2_init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    gpio.GPIO_Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C2_GPIO, &gpio);

    I2C_DeInit(I2C2);
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Init(I2C2, &i2c);
    I2C_Cmd(I2C2, ENABLE);
}
/* **********硬件相关初始化完成********** */

/* **********基础函数--I2C写入数据到设备的某寄存器****************** */
static uint8_t i2c2_write_reg(uint8_t dev_addr, uint16_t reg, uint8_t data)
{
    uint32_t timeout;
    uint8_t ret = VL53LXX_OK;

    I2C_LOCK();

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_Send7bitAddress(I2C2, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR)) {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF)) { ret = VL53LXX_ERR_I2C; goto done; }
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }
    if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF)) { ret = VL53LXX_ERR_I2C; goto done; }
    (void)I2C2->SR1;
    (void)I2C2->SR2;

    I2C_SendData(I2C2, (uint8_t)(reg >> 8));
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE)) {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF)) { ret = VL53LXX_ERR_I2C; goto done; }
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_SendData(I2C2, (uint8_t)(reg & 0xFF));
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE)) {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF)) { ret = VL53LXX_ERR_I2C; goto done; }
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_SendData(I2C2, data);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_BTF)) {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF)) { ret = VL53LXX_ERR_I2C; goto done; }
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_GenerateSTOP(I2C2, ENABLE);

done:
    I2C_UNLOCK();
    return ret;
}
/* **********基础函数--I2C读取设备的某寄存器数据****************** */
static uint8_t i2c2_read_reg(uint8_t dev_addr, uint16_t reg, uint8_t *data)
{
    uint32_t timeout;
    uint8_t ret = VL53LXX_OK;

    I2C_LOCK();

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_Send7bitAddress(I2C2, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }
    (void)I2C2->SR2;

    I2C_SendData(I2C2, (uint8_t)(reg >> 8));
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_SendData(I2C2, (uint8_t)(reg & 0xFF));
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_BTF)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_Send7bitAddress(I2C2, dev_addr << 1, I2C_Direction_Receiver);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    (void)I2C2->SR2;
    I2C_GenerateSTOP(I2C2, ENABLE);

    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }
    *data = I2C_ReceiveData(I2C2);

    I2C_AcknowledgeConfig(I2C2, ENABLE);

done:
    I2C_UNLOCK();
    return ret;
}
/* **********基础函数--I2C读取设备的某寄存器数据多个字节数据****************** */
static uint8_t i2c2_read_multi(uint8_t dev_addr, uint16_t reg, uint8_t *pdata, uint8_t count)
{
    uint32_t timeout;
    uint8_t ret = VL53LXX_OK;

    if (count == 0) return VL53LXX_OK;

    I2C_LOCK();

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_Send7bitAddress(I2C2, dev_addr << 1, I2C_Direction_Transmitter);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }
    (void)I2C2->SR2;// clear SR2; 读SB，ADDR，BTF，RXNE，TXE，AF位之后需要清除sr2

    I2C_SendData(I2C2, (uint8_t)(reg >> 8));
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_SendData(I2C2, (uint8_t)(reg & 0xFF));
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_BTF)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }

    I2C_Send7bitAddress(I2C2, dev_addr << 1, I2C_Direction_Receiver);
    timeout = VL53LXX_I2C_TIMEOUT;
    while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR)) {
        if (--timeout == 0) { ret = VL53LXX_ERR_I2C; goto done; }
    }
    (void)I2C2->SR2;

    while (count) {
        if (count == 1) {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
        }
        timeout = VL53LXX_I2C_TIMEOUT;
        while (!I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE)) {
            if (--timeout == 0) {
                I2C_GenerateSTOP(I2C2, ENABLE);
                I2C_AcknowledgeConfig(I2C2, ENABLE);
                ret = VL53LXX_ERR_I2C;
                goto done;
            }
        }
        *pdata++ = I2C_ReceiveData(I2C2);
        count--;
    }

    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

done:
    I2C_UNLOCK();
    return ret;
}
/* **********vl53lxx_功能封装--写入字节数据****************** */
static uint8_t vl53lxx_wr_byte(uint16_t reg, uint8_t data)
{
    return i2c2_write_reg(VL53LXX_DEFAULT_ADDR, reg, data);
}
/* **********vl53lxx_功能封装--读取字节数据****************** */
static uint8_t vl53lxx_rd_byte(uint16_t reg, uint8_t *data)
{
    return i2c2_read_reg(VL53LXX_DEFAULT_ADDR, reg, data);
}
/* **********vl53lxx_功能封装--读取16位数据****************** */
static uint8_t vl53lxx_rd_word(uint16_t reg, uint16_t *data)
{
    uint8_t buf[2];
    uint8_t ret;
    ret = i2c2_read_multi(VL53LXX_DEFAULT_ADDR, reg, buf, 2);
    if (ret == VL53LXX_OK) {
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}
/* **********vl53lxx_功能封装--读取多个字节数据****************** */
static uint8_t vl53lxx_rd_multi(uint16_t reg, uint8_t *pdata, uint8_t count)
{
    return i2c2_read_multi(VL53LXX_DEFAULT_ADDR, reg, pdata, count);
}
/* **********vl53lxx_功能封装--读取引导状态****************** */
static uint8_t vl53lxx_boot_state(uint8_t *state)
{
    return vl53lxx_rd_byte(VL53LXX_REG_FIRMWARE_SYSTEM_STATUS, state);
}
/* **********vl53lxx_功能封装--检查数据是否准备就绪****************** */
uint8_t vl53lxx_check_for_data_ready(uint8_t *is_ready)
{
    uint8_t temp, polarity;
    uint8_t ret;

    ret = vl53lxx_rd_byte(VL53LXX_REG_GPIO_HV_MUX_CTRL, &polarity);
    if (ret) return ret;
    polarity = !((polarity & 0x10) >> 4);

    ret = vl53lxx_rd_byte(VL53LXX_REG_GPIO_TIO_HV_STATUS, &temp);
    if (ret) return ret;

    *is_ready = ((temp & 0x01) == polarity) ? 1 : 0;
    return VL53LXX_OK;
}
/* **********vl53lxx_功能封装--初始化传感器****************** */
static uint8_t vl53lxx_sensor_init(void)
{
    uint8_t ret;
    uint8_t addr;
    uint8_t tmp = 0;
    uint16_t config_size = sizeof(VL51L1X_DEFAULT_CONFIG);
    uint32_t dbg_cnt = 0;

    for (addr = 0; addr < config_size; addr++) {
        ret = vl53lxx_wr_byte(0x2D + addr, VL51L1X_DEFAULT_CONFIG[addr]);
        if (ret) return ret;
    }

    ret = vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_START_RANGING);
    if (ret) return ret;

    while (tmp == 0) {
        ret = vl53lxx_check_for_data_ready(&tmp);
        if (ret) return ret;
        if (++dbg_cnt > 500) {
            return VL53LXX_ERR_TIMEOUT;
        }
        delay_us(1000);
    }

    tmp = 0;
    ret = vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (ret) return ret;

    ret = vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_STOP);
    if (ret) return ret;

    ret = vl53lxx_wr_byte(VL53LXX_REG_VHV_CONFIG_TIMEOUT_MACROP_BOUND, 0x09);
    if (ret) return ret;

    ret = vl53lxx_wr_byte(0x0B, 0x00);
    if (ret) return ret;

    return VL53LXX_OK;
}
/* **********vl53lxx_功能封装--初始化VL53LXX****************** */
uint8_t vl53lxx_init(void)
{
    uint8_t ret;
    uint8_t sensor_state = 0;
    uint16_t model_id;
    uint32_t timeout;

    xshut_init();
    delay_us(100);

    i2c2_init();

    if (g_i2c2_mutex == NULL) {
        g_i2c2_mutex = xSemaphoreCreateMutex();
    }

    xshut_set(1);
    delay_us(2000);

    ret = vl53lxx_rd_word(VL53LXX_REG_IDENTIFICATION_MODEL_ID, &model_id);
    if (ret) return VL53LXX_ERR_I2C;

    if (model_id != VL53LXX_MODEL_ID_EXPECTED) {
        return VL53LXX_ERR_ID;
    }

    timeout = 500;
    while (sensor_state == 0) {
        ret = vl53lxx_boot_state(&sensor_state);
        if (ret) return VL53LXX_ERR_I2C;
        if (--timeout == 0) {
            return VL53LXX_ERR_TIMEOUT;
        }
        delay_us(2000);
    }

    ret = vl53lxx_sensor_init();
    if (ret) {
        return ret;
    }

    return VL53LXX_OK;
}
/* **********vl53lxx_功能封装--设置定时预算****************** */
static void vl53lxx_set_timing_budget_internal(uint16_t ms)
{
    if (g_distance_mode == VL53LXX_DISTANCEMODE_SHORT) {
        switch (ms) {
        case 15:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x01);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0x1D);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x27);
            break;
        case 20:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0x51);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x6E);
            break;
        case 33:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xD6);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x6E);
            break;
        case 50:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x01);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xAE);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x01);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0xE8);
            break;
        case 100:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x02);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xE1);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x03);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x88);
            break;
        case 200:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x03);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xE1);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x04);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x96);
            break;
        case 500:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x05);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0x91);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x05);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0xC1);
            break;
        default:
            break;
        }
    } else {
        switch (ms) {
        case 20:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0x1E);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x22);
            break;
        case 33:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0x60);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0x6E);
            break;
        case 50:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xAD);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x00);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0xC6);
            break;
        case 100:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x01);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xCC);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x01);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0xEA);
            break;
        case 200:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x02);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0xD9);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x02);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0xF8);
            break;
        case 500:
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x04);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI + 1, 0x8F);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x04);
            vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI + 1, 0xA4);
            break;
        default:
            break;
        }
    }
}

void vl53lxx_set_timing_budget_ms(uint16_t ms)
{
    if (ms <= 15) ms = 15;
    else if (ms <= 20) ms = 20;
    else if (ms <= 33) ms = 33;
    else if (ms <= 50) ms = 50;
    else if (ms <= 100) ms = 100;
    else if (ms <= 200) ms = 200;
    else ms = 500;

    if (g_distance_mode == VL53LXX_DISTANCEMODE_SHORT && ms < 20) {
        ms = 20;
    }

    vl53lxx_set_timing_budget_internal(ms);
}

/* **********vl53lxx_功能封装--设置距离模式****************** */
void vl53lxx_set_distance_mode(uint8_t mode)
{
    uint8_t ready;
    uint32_t timeout;

    g_distance_mode = mode;

    if (mode == VL53LXX_DISTANCEMODE_SHORT) {
        vl53lxx_wr_byte(VL53LXX_REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
        vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
        vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
        vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_WOI_SD0, 0x07);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_WOI_SD0 + 1, 0x05);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_INITIAL_PHASE_SD0, 0x06);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_INITIAL_PHASE_SD0 + 1, 0x06);
    } else {
        vl53lxx_wr_byte(VL53LXX_REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A);
        vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
        vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
        vl53lxx_wr_byte(VL53LXX_REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_WOI_SD0, 0x0F);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_WOI_SD0 + 1, 0x0D);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_INITIAL_PHASE_SD0, 0x0E);
        vl53lxx_wr_byte(VL53LXX_REG_SD_CONFIG_INITIAL_PHASE_SD0 + 1, 0x0E);
    }

    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_START_RANGING);
    ready = 0;
    timeout = 500;
    while (!ready && timeout) {
        vl53lxx_check_for_data_ready(&ready);
        delay_us(1000);
        timeout--;
    }
    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_STOP);
}

/* **********vl53lxx_功能封装--启动连续模式****************** */
void vl53lxx_start_continuous(void)
{
    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_START_RANGING);
}

/* **********vl53lxx_功能封装--停止连续模式****************** */
void vl53lxx_stop_continuous(void)
{
    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_STOP);
}
/* **********vl53lxx_功能封装--检查数据→一次burst读取→解析结果→清中断→重启测量****************** */
static uint8_t vl53lxx_remap_range_status(uint8_t raw_rs)
{
    switch (raw_rs) {
    case 9:  return 0;
    case 6:  return 1;
    case 4:  return 2;
    case 8:  return 3;
    case 5:  return 4;
    case 3:  return 5;
    case 19: return 6;
    case 7:  return 7;
    case 12: return 9;
    case 18: return 10;
    case 22: return 11;
    case 23: return 12;
    case 13: return 13;
    default: return 255;
    }
}

uint8_t vl53lxx_read_result(vl53lxx_result_t *result)
{
    uint8_t temp8, is_ready;
    uint16_t temp16;
    uint8_t ret;

    ret = vl53lxx_check_for_data_ready(&is_ready);
    if (ret) return 0;
    if (!is_ready) return 0;

    ret = vl53lxx_rd_byte(VL53LXX_REG_RESULT_RANGE_STATUS, &temp8);
    if (ret) return 0;
    result->range_status = vl53lxx_remap_range_status(temp8 & 0x1F);

    ret = vl53lxx_rd_word(VL53LXX_REG_RESULT_PEAK_SIGNAL_RATE_MCPS_SD0, &temp16);
    if (ret) return 0;
    result->signal_rate = temp16 * 8;

    ret = vl53lxx_rd_word(VL53LXX_REG_RESULT_AMBIENT_RATE_MCPS_SD, &temp16);
    if (ret) return 0;
    result->ambient_rate = temp16 * 8;

    ret = vl53lxx_rd_word(VL53LXX_REG_RESULT_FINAL_CROSSTALK_RANGE_MM_SD0, &temp16);
    if (ret) return 0;

    if (temp16 > VL53LXX_OFFSET_MM) {
        result->distance_mm = temp16 - VL53LXX_OFFSET_MM;
    } else {
        result->distance_mm = 0;
    }

    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return 1;
}


// dump results
void vl53lxx_dump_results(void)
{
    uint8_t buf[44];
    uint8_t ret;
    uint8_t i;

    ret = i2c2_read_multi(VL53LXX_DEFAULT_ADDR, 0x0088, buf, 44);
    if (ret != VL53LXX_OK) {
        printf("  dump FAIL\n");
        return;
    }

    if (!(buf[0] & 0x07)) {
        printf("  dump SKIP (no data ready)\n");
        return;
    }

    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl53lxx_wr_byte(VL53LXX_REG_SYSTEM_MODE_START, VL53LXX_MODE_START_RANGING);

    printf("  [0x0088-0x00B3] raw hex:\n  ");
    for (i = 0; i < 44; i++) {
        printf("%02X ", buf[i]);
        if ((i + 1) % 16 == 0 && i < 43) printf("\n  ");
    }
    printf("\n");

    printf("  int_status   = 0x%02X\n", buf[0] & 0x3F);
    printf("  range_status = 0x%02X\n", buf[1]);
    printf("  stream_cnt   = %u\n", buf[3]);
    printf("  spad_cnt     = %u\n", ((uint16_t)buf[4] << 8) | buf[5]);
    {
        uint16_t r0 = ((uint16_t)buf[6] << 8) | buf[7];
        uint16_t r1 = ((uint16_t)buf[8] << 8) | buf[9];
        uint16_t r2 = ((uint16_t)buf[10] << 8) | buf[11];
        uint16_t r3 = ((uint16_t)buf[12] << 8) | buf[13];
        uint16_t distance = ((uint16_t)buf[14] << 8) | buf[15];
        printf("  signal_rate  = %u kcps\n", r0);
        printf("  ambient_rate = %u kcps\n", r1);
        printf("  sigma_mm     = %u.%02u mm\n", r2 / 256, r2 * 100 / 256);
        printf("  phase        = %u\n", r3);
        printf("  distance_raw = %u mm (0x%04X)\n", distance, distance);
    }
}
