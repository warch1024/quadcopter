
/***********************************************************************
 * 
 * 文件名   xn297l_remote.ino
 * 作者     hche
 * E-mail   
 * 日期     2023/07/08
 * 版本     V0.1.0
 * 说明     xn297l无线遥控实验
 * 
 ************************************************************************
 * 金华市核芯风暴智能科技有限公司
 ************************************************************************/
 

#include "car_ctrl.h"
#include <avr/wdt.h>
#include "hu_m40.h"

int PSS_CMD_prev=0xFF;

// Reset func
void (*resetFunc)(void) = 0;

void setup() {

  wdt_enable(WDTO_2S); //开启看门狗，并设置溢出时间为两秒

//  Serial.begin(115200);
//  Serial.print("system start\r\n");

  car_init();
  
  hu_m40_init();

}

void loop() {

  wdt_reset(); //喂狗操作，使看门狗定时器复位，至少2秒调用一次

  if (hu_m40_read() == 1)
  {
    uint8_t ly = 255-hu_m40_analog(HU_LY);
    uint8_t lx = hu_m40_analog(HU_LX);
    uint8_t ry = 255-hu_m40_analog(HU_RY);
    uint8_t rx = hu_m40_analog(HU_RX);

    int m = ry / 86;
    int n = rx / 86;
    int PSS_CMD;
    
    if (m == 0 && n == 1) PSS_CMD = 0;  //前进↑

    else if (m == 2 && n == 1) PSS_CMD = 1;  //后退↓

    else if (m == 1 && n == 0) PSS_CMD = 2;  //左平移←

    else if (m == 1 && n == 2) PSS_CMD = 3;  //右平移→

    else if (m == 0 && n == 0) PSS_CMD = 4;  //左上方移动↖

    else if (m == 0 && n == 2) PSS_CMD = 5;  //右上方移动↗

    else if (m == 2 && n == 0) PSS_CMD = 6;  //左下方移动↙

    else if (m == 2 && n == 2) PSS_CMD = 7;  //右下方移动↘

    else {

      m = ly / 86;
      n = lx / 86;

      if (m == 0 && n == 1) PSS_CMD = 0;  //前进↑

      else if (m == 2 && n == 1) PSS_CMD = 1;  //后退↓

      else if (m == 1 && n == 0) PSS_CMD = 2;  //左平移←

      else if (m == 1 && n == 2) PSS_CMD = 3;  //右平移→

      else if (m == 0 && n == 0) PSS_CMD = 4;  //左上方移动↖

      else if (m == 0 && n == 2) PSS_CMD = 5;  //右上方移动↗

      else if (m == 2 && n == 0) PSS_CMD = 6;  //左下方移动↙

      else if (m == 2 && n == 2) PSS_CMD = 7;  //右下方移动↘

      else PSS_CMD = 255;


      if( PSS_CMD != 255 )
      {
        ry = ly; // 因为下面动作是和ry rx相关，在这里要重新赋值
        rx = lx;
        

      }
    }

    if (hu_m40_button(HU_K1))  //原地左转
    {
      PSS_CMD = 8;
    }
    if (hu_m40_button(HU_K2))  //原地右转
    {
      PSS_CMD = 9;
    }
    if (hu_m40_button(HU_K3))  //朝向左移
    {
      PSS_CMD = 10;
    }
    if (hu_m40_button(HU_K4))  //朝向右移
    {
      PSS_CMD = 11;
    }

    if (PSS_CMD <= 11) {
      // Serial.print("PSS_CMD: ");
      // Serial.print(PSS_CMD);
      // Serial.print(", ry: ");
      // Serial.print(ry);
      // Serial.print(", rx: ");
      // Serial.print(rx);
      // Serial.print(", m: ");
      // Serial.print(m);
      // Serial.print(", n: ");
      // Serial.println(n);
    }

    switch (PSS_CMD) {
      case 0: // 后退
        car_back(255 - ry * 3);
        break;
      case 1: // 前进
        car_run(3 * ry - 516);
        break;
      case 2: // 右平移
        car_move_right(255 - rx * 3);
        break;
      case 3: // 左平移
        car_move_left(3 * rx - 516);
        break;
      case 4: // 左前
        if( (255 - ry * 3) > (255 - rx * 3) )
        {
          car_TR(255 - ry * 3);
        }
        else
        {
          car_TR(255 - rx * 3);
        }
        break;
      case 5: // 右前
        if( (255 - ry * 3) > (3 * rx - 516) )
        {
          car_TL(255 - ry * 3);
        }
        else
        {
          car_TL(3 * rx - 516);
        }
        break;
      case 6: // 左后
      
        if( (3 * ry - 516) > (255 - rx * 3) )
        {
          car_BR((3 * ry - 516));
        }
        else
        {
          car_BR(255 - rx * 3);
        }
        break;
      case 7: // 右后
        if( (3 * ry - 516) > (3 * rx - 516) )
        {
          car_BL((3 * ry - 516));
        }
        else
        {
          car_BL(3 * rx - 516);
        }
        break;

      case 8:  //原地左转
        car_spin_left(128);
        break;
      case 9:  //原地右转
        car_spin_right(128);
        break;
      case 10:  // 左转
        car_left(128);
        break;

      case 11:  // 右转
        car_right(128);
        break;


      default:
        
        if( PSS_CMD_prev != PSS_CMD )
        {
          car_brake();
        }
        else
        {
          car_stop();
        }
        
        break;
    }

    PSS_CMD_prev = PSS_CMD;

  } else {
    car_stop();
//    resetFunc();
  }

  delay(50);
}
