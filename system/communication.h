#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

#define HU_M40_ADDR_DEF             "HXFB0"
#define HU_M40_ADDR_ACK             "HXFB1"
#define HU_M40_PAYLOAD_WIDTH        16

#define HU_K1   0x0001
#define HU_K2   0x0002
#define HU_K3   0x0004
#define HU_K4   0x0008
#define HU_K5   0x0010
#define HU_K6   0x0020

#define HU_RX   4
#define HU_RY   5
#define HU_LX   6
#define HU_LY   7

/* ── RC 遥控数据队列（RC 任务 → FC 任务）── */
typedef struct {
    float   stick_roll;     // RX 通道原始值 0~255
    float   stick_pitch;    // RY 通道原始值 0~255
    float   stick_yaw;      // LX 通道原始值 0~255
    float   stick_throttle; // LY 通道原始值 0~255
    uint8_t buttons;        // 按键位掩码
    uint8_t rc_alive;       // 遥控器连接存活标志
    TickType_t timestamp;   // 收包时刻
} rc_input_t;

extern QueueHandle_t g_rc_queue;

extern uint8_t  g_rc_data[16];
extern uint8_t  g_rc_connected;
extern uint8_t  g_rx_addr[5];
extern uint8_t  g_tx_addr[5];

uint8_t hu_m40_checksum(uint8_t *buf, uint8_t len);
uint8_t hu_m40_receive_frame(uint8_t *buf);
void hu_m40_send_ack(uint8_t* dat);

void vTaskCommunicationTest(void *pvParameters);
void vTaskRCReceiver(void *pvParameters);

#endif
