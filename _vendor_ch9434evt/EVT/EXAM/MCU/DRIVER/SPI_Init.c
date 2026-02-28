/********************************** (C) COPYRIGHT *******************************
 * File Name          : SPI_Init.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : SPI driver file.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "SPI_Init.h"
#define  DATA_START 	 GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define  DATA_END	     GPIO_SetBits(GPIOA,GPIO_Pin_4)

void CH9434_US_DELAY(void)
{
    Delay_Us(1);
}

void CH9434_SPI_SCS_OP(u8_t dat)
{
    dat ?  DATA_END  :  DATA_START;
}


/*********************************************************************
 * @fn      SPI_FullDuplex_Init
 *
 * @brief   Configuring the SPI for full-duplex communication.
 *
 * @return  none
 */
void SPI_FullDuplex_Init(uint16_t SPI_BaudRatePrescaler_X)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef SPI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );

    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_X;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}


void CH9434_SPI_Init()
{
    SPI_FullDuplex_Init(SPI_BaudRatePrescaler_256);
}

void CH9434_INT_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}



u8_t CH9434_SPI_WRITE_BYTE(u8_t dat)
{
    SPI1->DATAR = dat;
    while(!(SPI1->STATR & SPI_I2S_FLAG_TXE));
    while(SPI1->STATR & SPI_I2S_FLAG_BSY);
    while(!(SPI1->STATR & SPI_I2S_FLAG_RXNE));

    return (u8_t)SPI1->DATAR;
}

