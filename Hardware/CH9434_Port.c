/**
  * @file    CH9434_Port.c
  * @brief   CH9434 官方驱动的硬件适配层
  *          实现 CH9434.h 中声明的 extern 函数，桥接到 STM32F103 SPI 底层
  */
#include "CH9434.h"
#include "SPI.h"
#include "Delay.h"
#include "stm32f10x.h"

/*******************************************************************************
 * CH9434 官方驱动需要的 3 个 extern 函数 (CH9434.h 中声明)
 ******************************************************************************/

/**
  * @brief  微秒级延时，官方驱动内部时序使用
  *         软件SPI时序抖动较大，需要比硬件SPI更长的延时
  */
void CH9434_US_DELAY(void)
{
    Delay_us(5);
}

/**
  * @brief  SPI 片选控制
  * @param  dat: CH9434_ENABLE(1)=CS拉高(释放), CH9434_DISABLE(0)=CS拉低(选中)
  * @note   官方驱动的约定: DISABLE=片选有效(低电平), ENABLE=片选无效(高电平)
  */
void CH9434_SPI_SCS_OP(u8_t dat)
{
    if (dat) {
        SPI_CS_HIGH();   /* CH9434_ENABLE -> CS 拉高，释放 */
    } else {
        SPI_CS_LOW();    /* CH9434_DISABLE -> CS 拉低，选中 */
    }
}

/**
  * @brief  SPI 读写一个字节（全双工）
  * @param  dat: 要发送的数据
  * @retval 同时接收到的数据
  */
u8_t CH9434_SPI_WRITE_BYTE(u8_t dat)
{
    return BSP_SPI_SwapByte(dat);
}
