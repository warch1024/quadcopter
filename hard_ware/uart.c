#include "uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"
#include "queue.h"
#include "string.h"
#include "stdio.h"

QueueHandle_t usart1_recvQueue = NULL;
QueueHandle_t usart1_sendQueue = NULL;

// 定义标准输出和输入流
struct __FILE __stdout;
struct __FILE __stdin;
int fputc(int ch, FILE* f)
{
    if (usart1_sendQueue != NULL) {
        uint8_t byte = (uint8_t)ch;
        xQueueSend(usart1_sendQueue, &byte, 0);  // 非阻塞：满则丢弃，绝不阻塞调用任务
    }
    return ch;
}
void _ttywrch(int ch)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET);
    USART_SendData(USART1, ch);
}

void _sys_exit(int return_code)
{
    label : goto label;
}


void uart1_init(uint32_t baudrate){
    // PA9     ------> USART1_TX
    // PA10    ------> USART1_RX
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    // USART1重映射到PB6/PB7，使能复用功能再重映射PB6/PB7为USART1_TX/RX
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = UART1_TX;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = UART1_RX;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 初始化使能USART1
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);// 使能USART1

    //接收中断配置
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    // 使能接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // 使能USART1
    USART_Cmd(USART1, ENABLE);

    usart1_recvQueue = xQueueCreate(UART1_RECV_QUEUE_SIZE, sizeof(uint8_t));   // 接收队列，用于存储接收的字节
    usart1_sendQueue = xQueueCreate(UART1_SEND_QUEUE_SIZE, sizeof(uint8_t));   // 发送队列，printf 非阻塞入队
}


void USART1_IRQHandler(void)    // 串口1的中断服务函数
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)   //判断是否是接收中断发生
    {
        UBaseType_t status = taskENTER_CRITICAL_FROM_ISR();
        // 释放临界区
        // // 释放接收信号量
        // xSemaphoreGiveFromISR(usart1_recvSemaphore, &status);
        uint8_t recv_byte = USART_ReceiveData(USART1); // 接收字节
        xQueueSendFromISR(usart1_recvQueue, &recv_byte, NULL);
        // printf("%c", recv_byte);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);  // 清除接收中断标志位

        taskEXIT_CRITICAL_FROM_ISR(status);
    }
}
//非阻塞接收任务
void task_usart1_recv(void *pvParameters){
    static uint8_t cmd_line[UART1_RECV_QUEUE_SIZE] = {0}, index = 0;
    while(1){
        // // 等待接收信号量
        // xSemaphoreTake(usart1_recvSemaphore, portMAX_DELAY);
        // // 处理接收中断
        // volatile uint8_t recv_byte = USART_ReceiveData(USART1); // 接收字节
        // printf("%c", recv_byte);
        xQueueReceive(usart1_recvQueue, &cmd_line[index++], portMAX_DELAY);
        if( cmd_line[index - 1] == '\n' || index >= sizeof(cmd_line) - 1){
            index = 0;
            printf("%s", cmd_line);
            memset(cmd_line, '\0', sizeof(cmd_line));
        }
    }
}
//非阻塞发送任务
void task_usart1_print(void *pvParameters){
    static uint8_t printf_buf[UART1_SEND_QUEUE_SIZE] = {0}, index = 0;
    while(1){
        xQueueReceive(usart1_sendQueue, &printf_buf[index++], portMAX_DELAY);
        if( printf_buf[index - 1] == '\n' || index >= sizeof(printf_buf) - 1){
            index = 0;
            uart1_send_str((char*)printf_buf);
            memset(printf_buf, '\0', sizeof(printf_buf));
        }
    }
}
