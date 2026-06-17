#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "queue.h"
#include "string.h"
#include "stdio.h"
#include "attitude.h"
#include "communication.h"
#include "electronic_speed_controller.h"


TaskHandle_t usart1_recvTaskHandle;
TaskHandle_t usart1_printTaskHandle;

void vTask1(void *pvParameters){

    GPIO_InitTypeDef GPIO_led_init;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   
    GPIO_led_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_led_init.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_led_init.GPIO_Pin = GPIO_Pin_13; //LED1
    GPIO_Init(GPIOC, &GPIO_led_init);

    while(1){
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        //printf("vTask1\n");

    }
}
//全局初始化函数
void init(){
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    uart1_init(2000000);
}

int main(void){
    init();
    xTaskCreate(vTask1, "vTask1", 128, NULL, 1, NULL);
    xTaskCreate(task_usart1_recv, "task_usart1_recv", 128, NULL, 3, &usart1_recvTaskHandle);
    xTaskCreate(task_usart1_print, "task_usart1_print", 128, NULL, 4, &usart1_printTaskHandle);
    // xTaskCreate(vTaskMPU6050Test, "vTaskMPU6050Test", 512, NULL, 5, NULL);
    // xTaskCreate(vTaskMahonyTest, "vTaskMahonyTest", 512, NULL, 5, NULL);
    // xTaskCreate(vTaskCommunicationTest, "vTaskComm", 512, NULL, 4, NULL);
    // xTaskCreate(vTaskESCControlTest, "vTaskESC", 256, NULL, 5, NULL);
    // xTaskCreate(vTaskVL53LXXDebugTest, "vTaskVL53", 512, NULL, 4, NULL);
    xTaskCreate(vTaskRCReceiver, "vTaskRC", 256, NULL, 6, NULL);   // RC 接收任务（高优先级）
    xTaskCreate(vTaskFCDebug, "vTaskFC", 512, NULL, 5, NULL);      // 飞控任务
    printf("Hello World!\n");
    vTaskStartScheduler();
    vTaskEndScheduler();
    
    
}


