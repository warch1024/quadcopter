#ifndef __UART_H__
#define __UART_H__
#include <stdio.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define UART1_TX  GPIO_Pin_9
#define UART1_RX  GPIO_Pin_10

extern QueueHandle_t usart1_sendQueue;

#define UART1_RECV_QUEUE_SIZE 32
#define UART1_SEND_QUEUE_SIZE 256    // 足够容纳一整行 debug printf，避免非阻塞丢字

struct __FILE
{
    int handle;
};



static inline int8_t uart1_send_byte(uint8_t byte)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);    // 等待发送完成
    USART_SendData(USART1, byte); // 发送字节
    // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);    // 等待发送完成
    return 1;
}

static inline int8_t uart1_send_str(char* str)
{
    while(*str != '\0'){
        uart1_send_byte(*str++);
    }
    return 1;
}


int fputc(int ch, FILE* f);
void _sys_exit(int return_code);

void uart1_init(uint32_t baudrate);

void task_usart1_recv(void *pvParameters);
void task_usart1_print(void *pvParameters);

#endif



