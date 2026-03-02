#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "UART.h"
#include "Key.h"
#include "MyCAN.h"      // STM32硬件CAN接收
#include "PMOD_CAN.h"   // PMOD发送CAN（官方CH9434D驱动）
#include "Timer.h"
#include "SPI.h"
#include <stdio.h>

uint8_t KeyNum;
volatile uint8_t TimingFlag;

/* PMOD发送的数据 */
CanTxMsg TxMsg_PMOD = {
	.StdId = 0x123,
	.ExtId = 0x00000000,
	.IDE = CAN_Id_Standard,
	.RTR = CAN_RTR_Data,
	.DLC = 4,
	.Data = {0x11, 0x22, 0x33, 0x44}
};

/* 硬件CAN接收的数据 */
CanRxMsg RxMsg;

int main(void)
{
	UART1_Init(115200);
	Key_Init();
	MyCAN_Init();
	PMOD_CAN_Init();
	Timer_Init();

	while (1)
	{
		/* 定时发送（通过PMOD发送CAN）*/
		if (TimingFlag == 1)
		{
			TimingFlag = 0;
			
			/* 更新发送数据 */
			TxMsg_PMOD.Data[0]++;
			TxMsg_PMOD.Data[1]++;
			TxMsg_PMOD.Data[2]++;
			TxMsg_PMOD.Data[3]++;
			
			uint8_t txResult = PMOD_CAN_Transmit(&TxMsg_PMOD);
			printf("[PMOD TX %s] ID:0x%03X Data: %02X %02X %02X %02X\r\n",
					txResult ? "OK" : "FAIL",
					TxMsg_PMOD.StdId,
					TxMsg_PMOD.Data[0], TxMsg_PMOD.Data[1],
					TxMsg_PMOD.Data[2], TxMsg_PMOD.Data[3]);
		}
		
		/* 按键触发发送 */
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			TxMsg_PMOD.Data[0] += 10;
			TxMsg_PMOD.Data[1] += 10;
			TxMsg_PMOD.Data[2] += 10;
			TxMsg_PMOD.Data[3] += 10;
			PMOD_CAN_Transmit(&TxMsg_PMOD);
			printf("[KEY TX] ID:0x%03X Data: %02X %02X %02X %02X\r\n",
					TxMsg_PMOD.StdId,
					TxMsg_PMOD.Data[0], TxMsg_PMOD.Data[1],
					TxMsg_PMOD.Data[2], TxMsg_PMOD.Data[3]);
		}
		
		/* 硬件CAN接收 */
		if (MyCAN_ReceiveFlag())
		{
			MyCAN_Receive(&RxMsg);
			
			if (RxMsg.RTR == CAN_RTR_Data)
			{
				printf("[HW CAN RX] ID:0x%03X DLC:%d Data: %02X %02X %02X %02X %02X %02X\r\n",
						RxMsg.StdId, RxMsg.DLC,
						RxMsg.Data[0], RxMsg.Data[1], RxMsg.Data[2], RxMsg.Data[3]);
			}
		}
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TimingFlag = 1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
