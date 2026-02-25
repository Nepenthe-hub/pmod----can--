#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f10x.h"

/* 函数声明 - STM32 硬件CAN接收 */
void MyCAN_Init(void);
void MyCAN_Transmit(CanTxMsg *TxMessage);
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(CanRxMsg *RxMessage);
uint8_t MyCAN_GetErrorStatus(void);
uint32_t MyCAN_GetLastError(void);

#endif
