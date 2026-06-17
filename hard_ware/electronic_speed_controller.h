#ifndef __ELECTRONIC_SPEED_CONTROLLER_H__
#define __ELECTRONIC_SPEED_CONTROLLER_H__

#include "stm32f10x.h"

#define ESC_PITCH_SCALE     500
#define ESC_ROLL_SCALE      500

void vTaskESCControlTest(void *pvParameters);
void vTaskVL53LXXDebugTest(void *pvParameters);
void vTaskFCDebug(void *pvParameters);

#endif
