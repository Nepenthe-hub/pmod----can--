#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"

/* 硬件引脚定义 (根据原理图修改) */
/* 假设使用 SPI1: PA5(SCK), PA6(MISO), PA7(MOSI) */
/* 片选 CS 使用 PA4 */
#define SPI_PORT            GPIOA
#define SPI_PIN_SCK         GPIO_Pin_5
#define SPI_PIN_MISO        GPIO_Pin_6
#define SPI_PIN_MOSI        GPIO_Pin_7
#define SPI_PIN_CS          GPIO_Pin_4

/* 简易的片选控制宏 */
#define SPI_CS_LOW()        GPIO_ResetBits(SPI_PORT, SPI_PIN_CS)
#define SPI_CS_HIGH()       GPIO_SetBits(SPI_PORT, SPI_PIN_CS)

/* 函数声明 */
void BSP_SPI_Init(void);
uint8_t BSP_SPI_SwapByte(uint8_t byte);

#endif
