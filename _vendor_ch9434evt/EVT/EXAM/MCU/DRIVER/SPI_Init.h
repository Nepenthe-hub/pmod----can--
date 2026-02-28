/********************************** (C) COPYRIGHT *******************************
 * File Name          : SPI_Init.h
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : SPI driver file.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef USER_SPI_INIT_H_
#define USER_SPI_INIT_H_
#include "CH9434.H"

#define  SPI_RECV_SIZE 4096
#define  SPI_SEND_SIZE 4096


extern uint8_t SPI_RECV[SPI_RECV_SIZE];
extern uint8_t SPI_SEND[SPI_SEND_SIZE];

void CH9434_SPI_Init();
void CH9434_INT_Init();
void SPI_FullDuplex_Init(uint16_t SPI_BaudRatePrescaler_X);
void SPI1_DMA_Init();
void SPI_DMA_WRITE_Data(uint8_t *Data ,uint16_t len);


#endif /* USER_SPI_INIT_H_ */
