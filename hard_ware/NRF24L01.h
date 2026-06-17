#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "stm32f10x.h"

#define NRF24L01_CE_PORT            GPIOB
#define NRF24L01_CE_PIN             GPIO_Pin_0
#define NRF24L01_CSN_PORT           GPIOA
#define NRF24L01_CSN_PIN            GPIO_Pin_4

#define NRF24L01_SPI               SPI1

#define CE_H()                      GPIO_SetBits(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
#define CE_L()                      GPIO_ResetBits(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
#define CSN_H()                     GPIO_SetBits(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)
#define CSN_L()                     GPIO_ResetBits(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)

#define NRF24L01_CMD_REGISTER_R     0x00
#define NRF24L01_CMD_REGISTER_W     0x20
#define NRF24L01_CMD_RX_PLOAD_R     0x61
#define NRF24L01_CMD_TX_PLOAD_W     0xA0
#define NRF24L01_CMD_FLUSH_TX       0xE1
#define NRF24L01_CMD_FLUSH_RX       0xE2
#define NRF24L01_CMD_REUSE_TX_PL    0xE3
#define NRF24L01_CMD_NOP            0xFF

#define NRF24L01_REG_CONFIG         0x00
#define NRF24L01_REG_EN_AA          0x01
#define NRF24L01_REG_EN_RXADDR      0x02
#define NRF24L01_REG_SETUP_AW       0x03
#define NRF24L01_REG_SETUP_RETR     0x04
#define NRF24L01_REG_RF_CH          0x05
#define NRF24L01_REG_RF_SETUP       0x06
#define NRF24L01_REG_STATUS         0x07
#define NRF24L01_REG_OBSERVE_TX     0x08
#define NRF24L01_REG_RPD            0x09
#define NRF24L01_REG_RX_ADDR_P0     0x0A
#define NRF24L01_REG_RX_ADDR_P1     0x0B
#define NRF24L01_REG_RX_ADDR_P2     0x0C
#define NRF24L01_REG_RX_ADDR_P3     0x0D
#define NRF24L01_REG_RX_ADDR_P4     0x0E
#define NRF24L01_REG_RX_ADDR_P5     0x0F
#define NRF24L01_REG_TX_ADDR        0x10
#define NRF24L01_REG_RX_PW_P0       0x11
#define NRF24L01_REG_RX_PW_P1       0x12
#define NRF24L01_REG_RX_PW_P2       0x13
#define NRF24L01_REG_RX_PW_P3       0x14
#define NRF24L01_REG_RX_PW_P4       0x15
#define NRF24L01_REG_RX_PW_P5       0x16
#define NRF24L01_REG_FIFO_STATUS    0x17
#define NRF24L01_REG_DYNPD          0x1C
#define NRF24L01_REG_FEATURE        0x1D

#define NRF24L01_CONFIG_PRIM_RX     0x01
#define NRF24L01_CONFIG_PWR_UP      0x02

#define NRF24L01_FLAG_RX_DREADY     0x40
#define NRF24L01_FLAG_TX_DSENT      0x20
#define NRF24L01_FLAG_MAX_RT        0x10

#define NRF24L01_MASK_REG_MAP       0x1F
#define NRF24L01_MASK_STATUS_IRQ    0x70

#define NRF24L01_ADDR_WIDTH         5
#define NRF24L01_PLOAD_WIDTH        16
#define NRF24L01_TEST_ADDR          "nRF24"

void NRF24L01_Init(void);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_Read_To_Buf(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t NRF24L01_Write_From_Buf(uint8_t reg, uint8_t *buf, uint8_t len);
void NRF24L01_RX_Mode(uint8_t *rx_addr, uint8_t *tx_addr);
void NRF24L01_TX_Mode(uint8_t *rx_addr, uint8_t *tx_addr);
uint8_t NRF24L01_RxPacket(uint8_t *rx_buf);
uint8_t NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len);
void NRF24L01_FlushRX(void);
void NRF24L01_FlushTX(void);
void NRF24L01_ClearIRQFlag(uint8_t reg);
void NRF24L01_ClearIRQFlags(void);

#endif
