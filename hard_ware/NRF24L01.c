#include "NRF24L01.h"
#include "stm32f10x.h"

static void spi1_init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;
    //72MHz
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = NRF24L01_CSN_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(NRF24L01_CSN_PORT, &gpio);

    gpio.GPIO_Pin = NRF24L01_CE_PIN;
    GPIO_Init(NRF24L01_CE_PORT, &gpio);

    CE_L();
    CSN_H();

    SPI_I2S_DeInit(NRF24L01_SPI);
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;    //72MHz/8=9MHz
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(NRF24L01_SPI, &spi);
    SPI_Cmd(NRF24L01_SPI, ENABLE);
}

/* ***************硬件设置—— spi1读写一个字节*************** */
static uint8_t spi1_readwrite_byte(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(NRF24L01_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(NRF24L01_SPI, data);
    while (SPI_I2S_GetFlagStatus(NRF24L01_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(NRF24L01_SPI);
}

void NRF24L01_Init(void)
{
    spi1_init();
    NRF24L01_Check();
}
/* ***************硬件设置—— 读取寄存器*************** */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t value;
    CSN_L();
    spi1_readwrite_byte(reg);
    value = spi1_readwrite_byte(NRF24L01_CMD_NOP);
    CSN_H();
    return value;
}
/* ***************硬件设置—— 写入寄存器*************** */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    CSN_L();
    if (reg < NRF24L01_CMD_REGISTER_W) {
        status = spi1_readwrite_byte(NRF24L01_CMD_REGISTER_W | (reg & NRF24L01_MASK_REG_MAP));
        spi1_readwrite_byte(value);
    } else {
        status = spi1_readwrite_byte(reg);
        if ((reg != NRF24L01_CMD_FLUSH_TX)
            && (reg != NRF24L01_CMD_FLUSH_RX)
            && (reg != NRF24L01_CMD_REUSE_TX_PL)
            && (reg != NRF24L01_CMD_NOP)) {
            spi1_readwrite_byte(value);
        }
    }
    CSN_H();
    return status;
}
/* ***************硬件设置—— 读取寄存器到缓冲区*************** */
uint8_t NRF24L01_Read_To_Buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status;
    CSN_L();
    status = spi1_readwrite_byte(reg);
    while (len--) {
        *buf++ = spi1_readwrite_byte(NRF24L01_CMD_NOP);
    }
    CSN_H();
    return status;
}
/* ***************硬件设置—— 从缓冲区写入寄存器*************** */
uint8_t NRF24L01_Write_From_Buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status;
    CSN_L();
    status = spi1_readwrite_byte(reg);
    while (len--) {
        spi1_readwrite_byte(*buf++);
    }
    CSN_H();
    return status;
}
/* ***************硬件设置—— 检查NRF24L01是否正常*************** */
uint8_t NRF24L01_Check(void)
{
    uint8_t rxbuf[5];
    uint8_t i;
    uint8_t *ptr = (uint8_t *)NRF24L01_TEST_ADDR;

    NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_TX_ADDR, ptr, 5);
    NRF24L01_Read_To_Buf(NRF24L01_CMD_REGISTER_R | NRF24L01_REG_TX_ADDR, rxbuf, 5);

    for (i = 0; i < 5; i++) {
        if (rxbuf[i] != *ptr++) return 1;
    }
    return 0;
}
/* ***************上层应用—— 清空接收队列*************** */
void NRF24L01_FlushRX(void)
{
    NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_RX, NRF24L01_CMD_NOP);
}
/* ***************上层应用—— 清空发送队列*************** */
void NRF24L01_FlushTX(void)
{
    NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
}
/* ***************上层应用—— 清除中断标志位*************** */
void NRF24L01_ClearIRQFlag(uint8_t reg)
{
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_STATUS, reg);
}
/* ***************上层应用—— 清除所有中断标志位*************** */
void NRF24L01_ClearIRQFlags(void)
{
    uint8_t reg;
    reg  = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    reg |= NRF24L01_MASK_STATUS_IRQ;
    NRF24L01_Write_Reg(NRF24L01_REG_STATUS, reg);
}
/* ***************上层应用—— 配置NRF24L01*************** */
static void _NRF24L01_Config(uint8_t *tx_addr)
{
    NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_TX_ADDR, tx_addr, NRF24L01_ADDR_WIDTH);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_RX_PW_P0, NRF24L01_PLOAD_WIDTH);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_RF_CH, 25);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_RF_SETUP, 0x25);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_SETUP_RETR, 0x88);
}
/* ***************上层应用—— 接收模式*************** */
void NRF24L01_RX_Mode(uint8_t *rx_addr, uint8_t *tx_addr)
{
    CE_L();
    _NRF24L01_Config(tx_addr);
    NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_RX_ADDR_P0, rx_addr, NRF24L01_ADDR_WIDTH);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_CONFIG, 0x0f);
    CE_H();
}
/* ***************上层应用—— 发送模式*************** */
void NRF24L01_TX_Mode(uint8_t *rx_addr, uint8_t *tx_addr)
{
    CE_L();
    _NRF24L01_Config(tx_addr);
    NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_RX_ADDR_P0, tx_addr, NRF24L01_ADDR_WIDTH);
    NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_CONFIG, 0x0e);
    CE_H();
}
/* ***************上层应用—— 接收一帧手柄的数据包*************** */
uint8_t NRF24L01_RxPacket(uint8_t *rx_buf)
{
    uint8_t status, result = 0;

    CE_L();
    status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);

    if (status & NRF24L01_FLAG_RX_DREADY) {
        NRF24L01_Read_To_Buf(NRF24L01_CMD_RX_PLOAD_R, rx_buf, NRF24L01_PLOAD_WIDTH);
        result = 1;
        NRF24L01_ClearIRQFlag(NRF24L01_FLAG_RX_DREADY);
    }
    CE_H();
    return result;
}
/* ***************上层应用—— 发送一帧手柄的数据包*************** */
uint8_t NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len)
{
    uint8_t status = 0x00;
    uint32_t timeout;

    CE_L();
    len = len > NRF24L01_PLOAD_WIDTH ? NRF24L01_PLOAD_WIDTH : len;
    NRF24L01_Write_From_Buf(NRF24L01_CMD_TX_PLOAD_W, tx_buf, len);
    CE_H();

    for (timeout = 0; timeout < 10000; timeout++);

    CE_L();
    status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    if (status & NRF24L01_FLAG_TX_DSENT) {
        NRF24L01_ClearIRQFlag(NRF24L01_FLAG_TX_DSENT);
    } else if (status & NRF24L01_FLAG_MAX_RT) {
        NRF24L01_FlushTX();
        NRF24L01_ClearIRQFlag(NRF24L01_FLAG_MAX_RT);
    }
    CE_H();
    return status;
}
