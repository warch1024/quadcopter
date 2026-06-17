#include "M8520.h"
#include "FreeRTOS.h"
#include "task.h"

void m8520_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef tim_oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 71;    //72MHz /72 = 1MHz 1us
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = 20000 - 1;    // 2w == 20ms，标准航模飞控频率50Hz
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tim_base);

    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse = M8520_PULSE_MIN;
    tim_oc.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &tim_oc);
    TIM_OC2Init(TIM2, &tim_oc);
    TIM_OC3Init(TIM2, &tim_oc);
    TIM_OC4Init(TIM2, &tim_oc);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}
/* ***************上层应用—— 设置单个电机的油门*************** */
void m8520_set_throttle(uint8_t channel, uint16_t pulse_us)
{
    if (pulse_us < M8520_PULSE_MIN) {
        pulse_us = M8520_PULSE_MIN;
    }
    if (pulse_us > M8520_PULSE_MAX) {
        pulse_us = M8520_PULSE_MAX;
    }

    switch (channel) {
    case M8520_CH1:
        TIM_SetCompare1(TIM2, pulse_us);
        break;
    case M8520_CH2:
        TIM_SetCompare2(TIM2, pulse_us);
        break;
    case M8520_CH3:
        TIM_SetCompare3(TIM2, pulse_us);
        break;
    case M8520_CH4:
        TIM_SetCompare4(TIM2, pulse_us);
        break;
    }
}

void m8520_unlock(void)
{
    m8520_set_throttle(M8520_CH1, M8520_PULSE_UNLOCK);
    m8520_set_throttle(M8520_CH2, M8520_PULSE_UNLOCK);
    m8520_set_throttle(M8520_CH3, M8520_PULSE_UNLOCK);
    m8520_set_throttle(M8520_CH4, M8520_PULSE_UNLOCK);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    m8520_set_throttle(M8520_CH1, M8520_PULSE_MIN);
    m8520_set_throttle(M8520_CH2, M8520_PULSE_MIN);
    m8520_set_throttle(M8520_CH3, M8520_PULSE_MIN);
    m8520_set_throttle(M8520_CH4, M8520_PULSE_MIN);
}
