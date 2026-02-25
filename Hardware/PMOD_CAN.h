#ifndef __PMOD_CAN_H
#define __PMOD_CAN_H

#include "stm32f10x.h"

/* PMOD CAN函数声明 */
void    PMOD_CAN_Init(void);
uint8_t PMOD_CAN_Transmit(CanTxMsg *TxMessage);  // 返回发送状态
uint8_t PMOD_CAN_GetMode(void);                  // 0x00=正常, 0x80=配置, 0x01=睡眠

#endif
