#ifndef __PMOD_CAN_H
#define __PMOD_CAN_H

#include "stm32f10x.h"

/* PMOD CAN函数声明 */
void    PMOD_CAN_Init(void);
uint8_t PMOD_CAN_Transmit(CanTxMsg *TxMessage);  // 返回发送状态
uint8_t PMOD_CAN_GetMode(void);                  // 0x00=正常, 0x80=配置, 0x01=睡眠
uint8_t PMOD_CAN_Receive(CanRxMsg *RxMessage);   // 返回接收状态
uint32_t PMOD_CAN_GetErrorReg(void);             // CH9434 CAN_ERRSR
uint32_t PMOD_CAN_GetTxStatusReg(void);          // CH9434 CAN_TSTATR
uint8_t PMOD_CAN_IsReady(void);                  // CH9434 初始化状态
uint8_t PMOD_CAN_ReadIOFuncRaw(uint8_t out4[4]); // CH9434 0x45 原始4字节

#endif
