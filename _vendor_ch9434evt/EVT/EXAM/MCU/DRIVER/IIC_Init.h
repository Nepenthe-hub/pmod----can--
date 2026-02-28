/********************************** (C) COPYRIGHT *******************************
 * File Name          : IIC_Init.H
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : IIC driver file.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef USER_IIC_INIT_H_
#define USER_IIC_INIT_H_
void CH9434_IIC_INIT(void);
void CH9434_IIC_START(void);
void CH9434_IIC_SEND_ADD(u8_t dir);
void CH9434_IIC_SEND_DATA(u8_t dat);
u8_t CH9434_IIC_READ_DATA(void);
void CH9434_IIC_STOP(void);
#endif /* USER_IIC_INIT_H_ */
