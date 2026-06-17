
/***********************************************************************
 * 
 * 文件名   g_hu_m40.c
 * 作者     hche
 * E-mail   
 * 日期     2023/07/08
 * 版本     V1.1.0
 * 说明     HU遥控手柄解码
 * 
 ************************************************************************
 * 金华市核芯风暴智能科技有限公司
 ***********************************************************************
 
 使用方法：
 
 // 初始化
 void setup() {
   hu_m40_init();
 }
 
 // 轮询
 void loop() {
   if (hu_m40_read() == 1) // ==1有收到数据
   {
     if( hu_m40_analog(HU_LX) > 200 ) // 左摇杆X值> 200
	 {
       // TODO
	 }
	 
     if (hu_m40_button(HU_K1))  // KEY1 按下
     {
       // TODO
     }
   }
 }
 
 */

#include "hu_m40.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN 0
#define CSN_PIN 1
#define PAYLOAD_WIDTH    16

RF24 radio(CE_PIN, CSN_PIN);

const uint8_t HU_M40_ADDR_DEF[5] = "HXFB0";

uint8_t g_hu_m40_rx_data[16];
uint32_t g_hu_m40_last_buttons;
uint32_t g_hu_m40_buttons;

uint8_t g_hu_m40_rx_addr[5] = "HXFB1";

uint8_t* g_hu_m40_tx_addr;

static unsigned long last_read;

uint8_t hu_m40_check_sum(uint8_t* buf, uint8_t len);
void hu_m40_ack(uint8_t* buf, uint8_t len);

uint8_t hu_m40_send_data(uint8_t* buf, uint8_t len);
uint8_t hu_m40_received_data(uint8_t* buf, uint8_t len);


/********************************************************
* 函数名    hu_m40_button
* 输入      button：可选
*          HU_K1
*          HU_K2
*          HU_K3
*          HU_K4
*          HU_K5
*          HU_K6
*           
* 输出      无
* 返回      1：按下
*           0: 松开
* 说明      获取按键状态
********************************************************/
uint8_t hu_m40_button(uint16_t button)
{
  return ((g_hu_m40_buttons & button) > 0);
}

/********************************************************
* 函数名    hu_m40_analog
* 输入      button：可选
*          HU_RX
*          HU_RY
*          HU_LX
*          HU_LY
* 输出      无
* 返回      模拟值
* 说明      获取摇杆状态
********************************************************/
uint8_t hu_m40_analog(uint8_t button)
{
   return g_hu_m40_rx_data[button];
}

/********************************************************
* 函数名    g_hu_m40_init
* 输入      无
* 输出      无
* 返回      1：初始化成功
*           0: 初始化失败
* 说明      初始化
********************************************************/
uint8_t hu_m40_init(void)
{
  uint8_t res;
  uint8_t i;
  
  delay(100);

  radio.begin();
  radio.openWritingPipe(HU_M40_ADDR_DEF);
  radio.openReadingPipe(0, HU_M40_ADDR_DEF);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(25);
  radio.setPayloadSize(PAYLOAD_WIDTH);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(0, true);
  radio.setRetries(8, 8);
  radio.setCRCLength(RF24_CRC_16);
  radio.startListening();
  g_hu_m40_tx_addr = (uint8_t*)HU_M40_ADDR_DEF;

  last_read = millis();
  
  for( i=0;i<16;i++)
  {
    g_hu_m40_rx_data[i] = 0;
  }

  g_hu_m40_rx_data[HU_RX] = 0x80;
  g_hu_m40_rx_data[HU_RY] = 0x80;
  g_hu_m40_rx_data[HU_LX] = 0x80;
  g_hu_m40_rx_data[HU_LY] = 0x80;

  g_hu_m40_buttons = 0;
  
  return res;
}


/********************************************************
* 函数名    read
* 输入      无
* 输出      无
* 返回      1：成功
*           0: 失败
* 说明      读取
********************************************************/
uint8_t hu_m40_read(void)
{
  uint8_t dat[PAYLOAD_WIDTH]={0};
  uint8_t tmp_buf[PAYLOAD_WIDTH]={0};
  uint8_t i;
  
  unsigned long ticks = millis() - last_read;
  static uint8_t res=1;
  uint8_t flag;
  
  if( ticks > 2000) // 通信超时
  {
    last_read = millis();

    hu_m40_init(); // 重新初始化

    res = 0;
  }

  if( radio.available() )
  {
    radio.read(dat, PAYLOAD_WIDTH);
    if( dat[0] == 1 && dat[1] == 3 && dat[15] == hu_m40_check_sum( dat, 15 ) ) // 校验成功
    {
      
      for( i=0;i<16;i++)
      {
        g_hu_m40_rx_data[i] = dat[i];
      }

      g_hu_m40_buttons = g_hu_m40_rx_data[8];
      
      res = 1;
      
      // 返回数据到遥控器
      dat[0] = 0x01; // 固定为1
      dat[1] = 0x83; // 功能码
      dat[2] = dat[2]; // 序号
      dat[3] = 11; // 长度
      dat[4] = g_hu_m40_rx_addr[0];
      dat[5] = g_hu_m40_rx_addr[1];
      dat[6] = g_hu_m40_rx_addr[2];
      dat[7] = g_hu_m40_rx_addr[3];
      dat[8] = g_hu_m40_rx_addr[4];
      dat[15] = hu_m40_check_sum( dat, 15 );
      
      radio.stopListening();
      
      radio.openWritingPipe(g_hu_m40_tx_addr);
      radio.openReadingPipe(0, g_hu_m40_tx_addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.setChannel(25);
      radio.setPayloadSize(PAYLOAD_WIDTH);
      radio.setDataRate(RF24_250KBPS);
      radio.setAutoAck(0, true);
      radio.setRetries(8, 8);
      radio.setCRCLength(RF24_CRC_16);
      radio.writeBlocking(dat, PAYLOAD_WIDTH, 1000);         //发送数据

      delay(5);
      
      radio.openWritingPipe(g_hu_m40_rx_addr);
      radio.openReadingPipe(0, g_hu_m40_rx_addr);
      radio.setPALevel(RF24_PA_MIN);
      radio.setChannel(25);
      radio.setPayloadSize(PAYLOAD_WIDTH);
      radio.setDataRate(RF24_250KBPS);
      radio.setAutoAck(0, true);
      radio.setRetries(8, 8);
      radio.setCRCLength(RF24_CRC_16);
      radio.startListening();
      g_hu_m40_tx_addr = g_hu_m40_rx_addr;

      last_read = millis();
    }
  }

  return res;
}

uint8_t hu_m40_check_sum(uint8_t* buf, uint8_t len)
{
  uint8_t i, sum;
  sum = 0;
  for( i=0;i<len;i++)
  {
    sum += buf[i];
  }

  return sum;
}
