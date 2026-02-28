#include "SPI.h"
#include "Delay.h"

/**
  * @brief  SPI 半时钟延时 (~2us)，匹配官方 EVT 的 Prescaler_256 (281kHz)
  *         官方硬件 SPI 每字节传输约 28us，软件模拟需要放慢速度
  */
static void BSP_SPI_DelayShort(void)
{
  Delay_us(2);
}

/**
  * @brief  初始化 SPI1 及其对应的 GPIO
  */
void BSP_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

  /* 1. 开启 GPIOB + AFIO 时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  /* 2. PB3 是 JTAG 的 JTDO，必须禁用 JTAG 才能当普通 GPIO
   *    SWJ_JTAGDisable: 关闭 JTAG 但保留 SWD (PA13/PA14)，释放 PB3/PB4/PA15
   *    重要: 请确保下载器使用 SWD 模式而非 JTAG 模式 */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

  /* 3. 配置 SCK(PB13) 和 MOSI(PB15) 为推挽输出 */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    /* 4. 配置 MISO(PB14) 为浮空输入 */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    /* 5. 配置 CS(PB3) 为推挽输出 */
    GPIO_InitStructure.GPIO_Pin = SPI_PIN_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_CS_PORT, &GPIO_InitStructure);

    /* 初始化空闲状态: CS高、SCK低、MOSI高 */
    SPI_CS_HIGH();
    GPIO_ResetBits(SPI_PORT, SPI_PIN_SCK);
    GPIO_SetBits(SPI_PORT, SPI_PIN_MOSI);
}

/**
  * @brief  SPI 发送并接收一个字节
  * @param  byte: 要发送的数据
  * @retval 接收到的数据
  */
uint8_t BSP_SPI_SwapByte(uint8_t byte)
{
  uint8_t rx = 0;

  for (uint8_t i = 0; i < 8; i++) {
    /* SCK 拉低 -> 放数据到 MOSI */
    GPIO_ResetBits(SPI_PORT, SPI_PIN_SCK);

    if (byte & 0x80) {
      GPIO_SetBits(SPI_PORT, SPI_PIN_MOSI);
    } else {
      GPIO_ResetBits(SPI_PORT, SPI_PIN_MOSI);
    }

    BSP_SPI_DelayShort();  /* 低电平保持 ~2us */

    /* SCK 拉高 -> 采样 MISO（上升沿） */
    GPIO_SetBits(SPI_PORT, SPI_PIN_SCK);

    BSP_SPI_DelayShort();  /* 等待数据稳定后再采样 */

    rx <<= 1;
    if (GPIO_ReadInputDataBit(SPI_PORT, SPI_PIN_MISO) == Bit_SET) {
      rx |= 0x01;
    }

    byte <<= 1;
  }

  GPIO_ResetBits(SPI_PORT, SPI_PIN_SCK);
  BSP_SPI_DelayShort();
  return rx;
}
 