#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"

/* 硬件引脚定义 — 根据原理图 PMOD 接口 (P4→P5) */
/* SCK=PB13, MISO=PB14, MOSI=PB15, CS=PB3(JTDO, 需禁用JTAG保留SWD) */
/* 注: PB4=INT(DB4), PB5=RST(DB5) */
#define SPI_PORT            GPIOB
#define SPI_PIN_SCK         GPIO_Pin_13
#define SPI_PIN_MISO        GPIO_Pin_14
#define SPI_PIN_MOSI        GPIO_Pin_15

#define SPI_CS_PORT         GPIOB
#define SPI_PIN_CS          GPIO_Pin_3

/* 简易的片选控制宏 */
#define SPI_CS_LOW()        GPIO_ResetBits(SPI_CS_PORT, SPI_PIN_CS)
#define SPI_CS_HIGH()       GPIO_SetBits(SPI_CS_PORT, SPI_PIN_CS)

/* 函数声明 */
void BSP_SPI_Init(void);
uint8_t BSP_SPI_SwapByte(uint8_t byte);

#endif
