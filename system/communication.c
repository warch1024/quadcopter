#include "communication.h"
#include "NRF24L01.h"
#include "cascade_PID.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "string.h"

uint8_t g_rx_addr[5] = HU_M40_ADDR_ACK;
uint8_t g_tx_addr[5] = HU_M40_ADDR_DEF;

uint8_t g_rc_data[16];
uint8_t g_rc_connected = 0;
QueueHandle_t g_rc_queue = NULL;

uint8_t hu_m40_checksum(uint8_t *buf, uint8_t len)
{
    uint8_t i, sum = 0;
    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

uint8_t hu_m40_receive_frame(uint8_t *buf)
{
    return NRF24L01_RxPacket(buf);
}
//传入接收到的正确的数据的index9-14，以及序号num（index2）
void hu_m40_send_ack(uint8_t* dat)
{
    dat[0] = 0x01; // 固定为1
    dat[1] = 0x83; // 功能码
    dat[2] = dat[2]; // 序号
    dat[3] = 11; // 长度
    dat[4] = g_rx_addr[0];
    dat[5] = g_rx_addr[1];
    dat[6] = g_rx_addr[2];
    dat[7] = g_rx_addr[3];
    dat[8] = g_rx_addr[4];
    dat[15] = hu_m40_checksum( dat, 15 );

    NRF24L01_TX_Mode(g_tx_addr, g_tx_addr);
    NRF24L01_TxPacket(dat, 16);
}

void vTaskCommunicationTest(void *pvParameters)
{
    uint8_t dat[16];
    uint8_t connected = 0;
    uint32_t tick_count = 0;
    uint32_t last_rx_tick = 0;

    NRF24L01_Init();
    NRF24L01_RX_Mode(g_tx_addr, g_tx_addr);

    printf("NRF24L01 init done, waiting for HU-M40...\n");

    while (1) {
        if (hu_m40_receive_frame(dat)) {
            if (dat[0] == 1 && dat[1] == 3 && dat[15] == hu_m40_checksum(dat, 15)) {
                last_rx_tick = xTaskGetTickCount();

                if (!connected) {
                    connected = 1;
                    g_rc_connected = 1;
                    printf(">>> HU-M40 CONNECTED <<<\n");
                }

                memcpy(g_rc_data, dat, 16);//保存数据到g_rc_data

                printf("CH: LY=%3u LX=%3u RY=%3u RX=%3u | BTN: K1=%d K2=%d K3=%d K4=%d K5=%d K6=%d\n",
                       dat[HU_LY], dat[HU_LX], dat[HU_RY], dat[HU_RX],
                       (dat[8] & HU_K1) ? 1 : 0,
                       (dat[8] & HU_K2) ? 1 : 0,
                       (dat[8] & HU_K3) ? 1 : 0,
                       (dat[8] & HU_K4) ? 1 : 0,
                       (dat[8] & HU_K5) ? 1 : 0,
                       (dat[8] & HU_K6) ? 1 : 0);

                hu_m40_send_ack(dat);   //在发送ack之前确保dat已经保存副本
                //之后转换为接收模式，等待遥控器发送下一帧数据
                NRF24L01_RX_Mode(g_rx_addr, g_rx_addr);
                memcpy(g_tx_addr, g_rx_addr, 5);    //更新tx_addr为rx_addr
            }
        }

        tick_count = xTaskGetTickCount();
        if (connected && (tick_count - last_rx_tick > 2000 / portTICK_PERIOD_MS)) {
            connected = 0;
            g_rc_connected = 0;
            printf(">>> HU-M40 DISCONNECTED <<<\n");
            NRF24L01_RX_Mode(HU_M40_ADDR_DEF, HU_M40_ADDR_DEF);
            memcpy(g_tx_addr, HU_M40_ADDR_DEF, 5);
        }

        vTaskDelay(8 / portTICK_PERIOD_MS);
    }
}

/* ── RC 接收任务：独立于飞控，通过消息队列向 FC 任务推送遥控数据 ── */
void vTaskRCReceiver(void *pvParameters)
{
    rc_input_t rc;
    uint8_t dat[16];
    uint8_t connected = 0;
    TickType_t last_rx_tick = 0;
    TickType_t now_tick;

    /* 创建 length=1 的队列，xQueueOverwrite 保证永远是最新数据 */
    g_rc_queue = xQueueCreate(1, sizeof(rc_input_t));
    configASSERT(g_rc_queue != NULL);

    NRF24L01_Init();
    NRF24L01_RX_Mode(g_tx_addr, g_tx_addr);
    printf("[RC] NRF24L01 init done, waiting for HU-M40...\n");

    while (1) {
        if (hu_m40_receive_frame(dat)) {
            if (dat[0] == 1 && dat[1] == 3 && dat[15] == hu_m40_checksum(dat, 15)) {
                now_tick = xTaskGetTickCount();
                last_rx_tick = now_tick;

                if (!connected) {
                    connected = 1;
                    g_rc_connected = 1;
                    printf("[RC] >>> HU-M40 CONNECTED <<<\n");
                }

                memcpy(g_rc_data, dat, 16);

                /* 组装遥控数据消息，写入队列（覆盖旧数据） */
                rc.stick_roll     = (float)dat[HU_RX];
                rc.stick_pitch    = (float)dat[HU_RY];
                rc.stick_yaw      = (float)dat[HU_LX];
                rc.stick_throttle = (float)dat[HU_LY];
                rc.buttons        = dat[8];
                rc.rc_alive       = 1;
                rc.timestamp      = now_tick;
                xQueueOverwrite(g_rc_queue, &rc);

                hu_m40_send_ack(dat);
                NRF24L01_RX_Mode(g_rx_addr, g_rx_addr);
                memcpy(g_tx_addr, g_rx_addr, 5);
            }
        }

        /* 超时判定：断连时推送 rc_alive=0 */
        now_tick = xTaskGetTickCount();
        if (connected && ((now_tick - last_rx_tick) * portTICK_PERIOD_MS) >= FC_RC_TIMEOUT_MS) {
            connected = 0;
            g_rc_connected = 0;
            printf("[RC] >>> HU-M40 DISCONNECTED <<<\n");

            rc.rc_alive = 0;
            rc.timestamp = now_tick;
            xQueueOverwrite(g_rc_queue, &rc);

            NRF24L01_RX_Mode(HU_M40_ADDR_DEF, HU_M40_ADDR_DEF);
            memcpy(g_tx_addr, HU_M40_ADDR_DEF, 5);
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
