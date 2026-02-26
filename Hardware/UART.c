#include "stm32f10x.h"                  // Device header
#include "UART.h"
#include <string.h>
#include <stdio.h>

// 简单的接收缓冲区
static uint8_t rx_buffer[64];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_flag = 0;

void UART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(USART1_GPIO_CLK | USART1_CLK, ENABLE);
    
    // 2. 配置GPIO引脚
    // PA9 - TX 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART1_GPIO_PORT, &GPIO_InitStructure);
    
    // PA10 - RX 浮空输入
    GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART1_GPIO_PORT, &GPIO_InitStructure);
    
    // 3. 配置USART参数
    USART_InitStructure.USART_BaudRate = baudrate;           // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;      // 无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    
    USART_Init(USART1_PERIPH, &USART_InitStructure);
    
    // 4. 使能接收中断（可选）
    USART_ITConfig(USART1_PERIPH, USART_IT_RXNE, ENABLE);
    
    // 5. 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 6. 使能USART
    USART_Cmd(USART1_PERIPH, ENABLE);
}

// 发送单个字节
void UART1_SendByte(uint8_t data)
{
    // 等待发送缓冲区为空
    while(USART_GetFlagStatus(USART1_PERIPH, USART_FLAG_TXE) == RESET);
    
    // 发送数据
    USART_SendData(USART1_PERIPH, data);
}

// 发送字符串
void UART1_SendString(char *str)
{
    while(*str) {
        UART1_SendByte(*str++);
    }
}

// 重定向 printf 到 UART1（Keil MicroLIB 需勾选 Use MicroLIB）
int fputc(int ch, FILE *f)
{
    UART1_SendByte((uint8_t)ch);
    return ch;
}

// 发送数据数组
void UART1_SendData(uint8_t *data, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++) {
        UART1_SendByte(data[i]);
    }
}

// 接收单个字节（阻塞方式）
uint8_t UART1_ReceiveByte(void)
{
    // 等待接收到数据
    while(USART_GetFlagStatus(USART1_PERIPH, USART_FLAG_RXNE) == RESET);
    
    // 返回接收到的数据
    return USART_ReceiveData(USART1_PERIPH);
}

// 检查是否有数据可用（非阻塞）
uint8_t UART1_DataAvailable(void)
{
    return USART_GetFlagStatus(USART1_PERIPH, USART_FLAG_RXNE) != RESET;
}

// 获取接收到的字符串（如果使用中断）
char* UART1_GetReceivedString(void)
{
    if(rx_flag) {
        rx_buffer[rx_index] = '\0';  // 添加字符串结束符
        rx_flag = 0;
        rx_index = 0;
        return (char*)rx_buffer;
    }
    return NULL;
}

// USART1中断服务函数
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1_PERIPH, USART_IT_RXNE) != RESET) {
        // 读取接收到的数据
        uint8_t data = USART_ReceiveData(USART1_PERIPH);
        
        // 简单的回显（可选）
        UART1_SendByte(data);
        
        // 存储到缓冲区（可选）
        if(rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = data;
            
            // 如果收到回车或换行，设置标志
            if(data == '\r' || data == '\n') {
                rx_flag = 1;
            }
        }
        
        // 清除中断标志
        USART_ClearITPendingBit(USART1_PERIPH, USART_IT_RXNE);
    }
}
