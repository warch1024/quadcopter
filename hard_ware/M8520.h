#ifndef __M8520_H__
#define __M8520_H__

#include "stm32f10x.h"

#define M8520_CH1           1
#define M8520_CH2           2
#define M8520_CH3           3
#define M8520_CH4           4

#define M8520_PULSE_MIN     1000
#define M8520_PULSE_MAX     2300    /* 自由飞最高油门 */
#define M8520_PULSE_UNLOCK  1100

void m8520_init(void);
void m8520_set_throttle(uint8_t channel, uint16_t pulse_us);
void m8520_unlock(void);

#endif
