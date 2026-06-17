
/***********************************************************************
 * 
 * 文件名   car_ctrl.c
 * 作者     hche
 * E-mail   
 * 日期     2023/07/08
 * 版本     V0.1.0
 * 说明     小车控制程序
 * 
 ************************************************************************
 * 金华市核芯风暴智能科技有限公司
 ************************************************************************/
 
#include "car_ctrl.h"
#include "SoftPWM.h"


#define M_INA2 4
#define M_INB2 3

#define M_INA1 6
#define M_INB1 5

#define M_INA4 8
#define M_INB4 7

#define M_INA3 10
#define M_INB3 9

/********************************************************
* 函数名    car_init
* 输入      无
* 输出      无
* 返回      无
* 说明      初始化
********************************************************/
void car_init(void)
{

  pinMode(M_INA1, OUTPUT);
  pinMode(M_INB1, OUTPUT);
  pinMode(M_INA2, OUTPUT);
  pinMode(M_INB2, OUTPUT);
  pinMode(M_INA3, OUTPUT);
  pinMode(M_INB3, OUTPUT);
  pinMode(M_INA4, OUTPUT);
  pinMode(M_INB4, OUTPUT);

  SoftPWMBegin();

}

void motor_pin_out( int8_t pin, uint8_t value)
{
  SoftPWMSet(pin, value);
}

/********************************************************
* 函数名    m1_out
* 输入      dir: 方向，1正转，-1反转，其他
*           speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      m1电机输出
********************************************************/
void m1_out( int8_t dir, uint8_t speed)
{
  if( dir == 1 )
  {
    motor_pin_out(M_INA2, 0);
    motor_pin_out(M_INB2, speed);
  }
  else
  if( dir == -1 )
  {
    motor_pin_out(M_INB2, 0);
    motor_pin_out(M_INA2, speed);
  }
  else
  {
    motor_pin_out(M_INA2, speed);
    motor_pin_out(M_INB2, speed);
  }
}

/********************************************************
* 函数名    m2_out
* 输入      dir: 方向，1正转，-1反转，其他
*           speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      m2电机输出
********************************************************/
void m2_out( int8_t dir, uint8_t speed)
{
  if( dir == 1 )
  {
    motor_pin_out(M_INB1, 0);
    motor_pin_out(M_INA1, speed);
  }
  else
  if( dir == -1 )
  {
    motor_pin_out(M_INA1, 0);
    motor_pin_out(M_INB1, speed);
  }
  else
  {
    motor_pin_out(M_INB1, speed);
    motor_pin_out(M_INA1, speed);
  }
}

/********************************************************
* 函数名    m3_out
* 输入      dir: 方向，1正转，-1反转，其他
*           speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      m3电机输出
********************************************************/
void m3_out( int8_t dir, uint8_t speed)
{
  if( dir == 1 )
  {
    motor_pin_out(M_INA4, 0);
    motor_pin_out(M_INB4, speed);
  }
  else
  if( dir == -1 )
  {
    motor_pin_out(M_INB4, 0);
    motor_pin_out(M_INA4, speed);
  }
  else
  {
    motor_pin_out(M_INA4, speed);
    motor_pin_out(M_INB4, speed);
  }
}

/********************************************************
* 函数名    m4_out
* 输入      dir: 方向，1正转，-1反转，其他
*           speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      m4电机输出
********************************************************/
void m4_out( int8_t dir, uint8_t speed)
{
  if( dir == 1 )
  {
    motor_pin_out(M_INB3, 0);
    motor_pin_out(M_INA3, speed);
  }
  else
  if( dir == -1 )
  {
    motor_pin_out(M_INA3, 0);
    motor_pin_out(M_INB3, speed);
  }
  else
  {
    motor_pin_out(M_INB3, speed);
    motor_pin_out(M_INA3, speed);
  }
}


/********************************************************
* 函数名    car_run
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      前进
********************************************************/
void car_run(uint8_t speed)
{
  m1_out(1,speed);
  m2_out(1,speed);
  m3_out(1,speed);
  m4_out(1,speed);
}

/********************************************************
* 函数名    car_back
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      后退
********************************************************/
void car_back(uint8_t speed)
{
  m1_out(-1,speed);
  m2_out(-1,speed);
  m3_out(-1,speed);
  m4_out(-1,speed);
}

/********************************************************
* 函数名    car_left
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      左转(左轮不动，右轮前进)
********************************************************/
void car_left(uint8_t speed)
{
  m1_out(0,0);
  m2_out(0,0);
  m3_out(1,speed);
  m4_out(1,speed);
}

/********************************************************
* 函数名    car_spin_left
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      左转(左轮后退，右轮前进)
********************************************************/
void car_spin_left(uint8_t speed)
{
  m1_out(-1,speed);
  m2_out(-1,speed);
  m3_out(1,speed);
  m4_out(1,speed);
}

/********************************************************
* 函数名    car_move_left
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      左平移
********************************************************/
void car_move_left(uint8_t speed)
{
  m1_out(-1,speed);
  m2_out(1,speed);
  m3_out(-1,speed);
  m4_out(1,speed);
}

/********************************************************
* 函数名    car_right
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      右转(右轮不动，左轮前进)
********************************************************/
void car_right(uint8_t speed)
{
  m1_out(1,speed);
  m2_out(1,speed);
  m3_out(0,speed);
  m4_out(0,speed);
}

/********************************************************
* 函数名    car_spin_right
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      右转(右轮后退，左轮前进)
********************************************************/
void car_spin_right(uint8_t speed)
{
  m1_out(1,speed);
  m2_out(1,speed);
  m3_out(-1,speed);
  m4_out(-1,speed);
}

/********************************************************
* 函数名    car_move_right
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      右平移
********************************************************/
void car_move_right(uint8_t speed)
{
  m1_out(1,speed);
  m2_out(-1,speed);
  m3_out(1,speed);
  m4_out(-1,speed);
}

/********************************************************
* 函数名    car_TL
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      左上移动↖
********************************************************/
void car_TL(uint8_t speed)
{
  m1_out(0,speed);
  m2_out(1,speed);
  m3_out(0,speed);
  m4_out(1,speed);
}

/********************************************************
* 函数名    car_TR
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      右上移动↗
********************************************************/
void car_TR(uint8_t speed)
{
  m1_out(1,speed);
  m2_out(0,speed);
  m3_out(1,speed);
  m4_out(0,speed);
}

/********************************************************
* 函数名    car_BL
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      左下移动↙
********************************************************/
void car_BL(uint8_t speed)
{
  m1_out(-1,speed);
  m2_out(0,speed);
  m3_out(-1,speed);
  m4_out(0,speed);
}

/********************************************************
* 函数名    car_BR
* 输入      speed: 速度，0~255
* 输出      无
* 返回      无
* 说明      右下移动↘
********************************************************/
void car_BR(uint8_t speed)
{
  m1_out(0,speed);
  m2_out(-1,speed);
  m3_out(0,speed);
  m4_out(-1,speed);
}

/********************************************************
* 函数名    car_brake
* 输入      无
* 输出      无
* 返回      无
* 说明      刹车制动
********************************************************/
void car_brake(void)
{
  m1_out(0,0xFF);
  m2_out(0,0xFF);
  m3_out(0,0xFF);
  m4_out(0,0xFF);

}

/********************************************************
* 函数名    car_stop
* 输入      无
* 输出      无
* 返回      无
* 说明      停止
********************************************************/
void car_stop(void)
{
  m1_out(0,0);
  m2_out(0,0);
  m3_out(0,0);
  m4_out(0,0);
}
