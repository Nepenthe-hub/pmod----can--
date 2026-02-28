#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f10x.h"

/*
 * 硬件CAN引脚选择：
 * 0 = 默认映射 PA11(RX) / PA12(TX)
 * 1 = 重映射到 PB8(RX) / PB9(TX)
 */
#ifndef MYCAN_USE_REMAP_PB8_PB9
#define MYCAN_USE_REMAP_PB8_PB9 1
#endif

/* 函数声明 - STM32 硬件CAN接收 */
void MyCAN_Init(void);
void MyCAN_Transmit(CanTxMsg *TxMessage);
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(CanRxMsg *RxMessage);
uint8_t MyCAN_GetErrorStatus(void);
uint32_t MyCAN_GetLastError(void);
uint32_t MyCAN_GetPendingCount(void);

#endif
