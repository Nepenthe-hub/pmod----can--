#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"

// USART1 引脚定义
#define USART1_GPIO_PORT    GPIOA
#define USART1_GPIO_CLK     RCC_APB2Periph_GPIOA
#define USART1_TX_PIN       GPIO_Pin_9
#define USART1_RX_PIN       GPIO_Pin_10

// USART1 外设定义
#define USART1_PERIPH       USART1
#define USART1_CLK          RCC_APB2Periph_USART1
#define USART1_IRQn         USART1_IRQn

// 函数声明
void UART1_Init(uint32_t baudrate);
void UART1_SendByte(uint8_t data);
void UART1_SendString(char *str);
void UART1_SendData(uint8_t *data, uint16_t length);
uint8_t UART1_ReceiveByte(void);
uint8_t UART1_DataAvailable(void);

#endif
