#include "SPI.h"

/**
  * @brief  初始化 SPI1 及其对应的 GPIO
  */
void BSP_SPI_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 1. 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    /* 2. 配置 SCK (PA5) 和 MOSI (PA7) 为复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    /* 3. 配置 MISO (PA6) 为浮空输入或上拉输入 */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 推荐上拉输入增强稳定性
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    /* 4. 配置 CS (PA4) 为通用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    /* 初始化片选状态为高 (不选中) */
    SPI_CS_HIGH();

    /* 5. SPI1 参数配置 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      // 主机模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  // 8位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         // 时钟极性: 空闲为低
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                       // 时钟相位: 第1个沿采样 (Mode 0)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          // 软件NSS
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 速率分频 (72M/8 = 9Mhz，CH9434支持高速SPI)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 // 高位在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    /* 使能 SPI */
    SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  SPI 发送并接收一个字节
  * @param  byte: 要发送的数据
  * @retval 接收到的数据
  */
uint8_t BSP_SPI_SwapByte(uint8_t byte)
{
    /* 等待发送缓冲区为空 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    
    /* 发送数据 */
    SPI_I2S_SendData(SPI1, byte);
    
    /* 等待接收完成 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    /* 返回接收到的数据 */
    return SPI_I2S_ReceiveData(SPI1);
}
 