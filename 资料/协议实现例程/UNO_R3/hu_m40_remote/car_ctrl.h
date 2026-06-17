
/***********************************************************************
 * 
 * 文件名   car_ctrl.h
 * 作者     hche
 * E-mail   
 * 日期     2023/07/08
 * 版本     V0.1.0
 * 说明     小车控制程序
 * 
 ************************************************************************
 * 金华市核芯风暴智能科技有限公司
 ************************************************************************/
 
#ifndef __CAR_CTRL_H
#define __CAR_CTRL_H

#if ARDUINO > 22
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

void car_init(void);
void car_run(uint8_t speed);
void car_back(uint8_t speed);
void car_left(uint8_t speed);
void car_spin_left(uint8_t speed);
void car_move_left(uint8_t speed);
void car_right(uint8_t speed);
void car_spin_right(uint8_t speed);
void car_move_right(uint8_t speed);
void car_TL(uint8_t speed);
void car_TR(uint8_t speed);
void car_BL(uint8_t speed);
void car_BR(uint8_t speed);
void car_brake(void);
void car_stop(void);

#endif
