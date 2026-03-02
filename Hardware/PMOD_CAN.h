#ifndef __PMOD_CAN_H
#define __PMOD_CAN_H

#include "stm32f10x.h"

/* PMOD CAN 函数声明 */
void     PMOD_CAN_Init(void);
uint8_t  PMOD_CAN_Transmit(CanTxMsg *TxMessage);
uint8_t  PMOD_CAN_Receive(CanRxMsg *RxMessage);
uint8_t  PMOD_CAN_IsReady(void);
uint8_t  PMOD_CAN_LoopbackTest(void);

/* 诊断辅助（可选） */
uint8_t  PMOD_CAN_GetMode(void);
uint32_t PMOD_CAN_GetErrorReg(void);
uint32_t PMOD_CAN_GetTxStatusReg(void);
uint8_t  PMOD_CAN_ReadIOFuncRaw(uint8_t out4[4]);
uint32_t PMOD_CAN_GetBitTimingReg(void);
uint8_t  PMOD_CAN_GetCANPinStatus(void);
uint8_t  PMOD_CAN_GetHSEPinStatus(void);

#endif
