/********************************** (C) COPYRIGHT *******************************
 * File Name          : IIC_Init.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : IIC driver file.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "CH9434.H"
#include "debug.h"

void CH9434_IIC_INIT (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitStructure = {0};

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_ClockSpeed = 400000;  // 400kHz
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init (I2C1, &I2C_InitStructure);
    I2C_Cmd (I2C1, ENABLE);
    I2C_AcknowledgeConfig (I2C1, ENABLE);

    CH9434IICAddSet (0);//Set this value based on the levels of the two IIC_ADDR pins of CH9434D.
}

void CH9434_IIC_START (void) {
    I2C_GenerateSTART (I2C1, ENABLE);
    while (!I2C_CheckEvent (I2C1, I2C_EVENT_MASTER_MODE_SELECT));
}

void CH9434_IIC_SEND_ADD (u8_t dir) {
    u8_t flag = (dir & 0x01);
    u16_t timeout = 0;
    I2C_Send7bitAddress (I2C1, dir, flag ? I2C_Direction_Receiver : I2C_Direction_Transmitter);
    while (!I2C_CheckEvent (I2C1, flag ? I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED : I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
}

void CH9434_IIC_SEND_DATA (u8_t dat) {
    u16_t timeout = 0;
    
    I2C_SendData (I2C1, dat);
    while (!I2C_CheckEvent (I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
}

u8_t CH9434_IIC_READ_DATA (void) {
    while (!I2C_GetFlagStatus (I2C1, I2C_FLAG_RXNE));
    u8_t data = I2C_ReceiveData (I2C1);
    return data;
}

void CH9434_IIC_STOP (void) {
    I2C_GenerateSTOP (I2C1, ENABLE);
    while (I2C1->CTLR1 & I2C_CTLR1_STOP);
}
