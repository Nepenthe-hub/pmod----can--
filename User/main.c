#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "UART.h"
#include "Key.h"
#include "MyCAN.h"      // STM32硬件CAN接收
#include "PMOD_CAN.h"   // PMOD发送CAN
#include "Timer.h"
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
	
	UART1_Init(115200);  // 初始化串口，波特率115200
	Key_Init();
	MyCAN_Init();       // 初始化STM32硬件CAN接收
	PMOD_CAN_Init();    // 初始化PMOD CAN发送
	Timer_Init();
	
	/* 串口显示初始化信息 */
	printf("\r\n========================================\r\n");
	printf("PMOD TX / Hardware CAN RX System\r\n");
	printf("========================================\r\n");
	printf("[CAN MAP] %s\r\n", MYCAN_USE_REMAP_PB8_PB9 ? "PB8/PB9" : "PA11/PA12");
	printf("[CH9434] Ready=%d Mode=0x%02X\r\n", PMOD_CAN_IsReady(), PMOD_CAN_GetMode());
	
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
			
			/* 通过PMOD发送，硬件CAN只做接收，不再发送 */
			uint8_t txResult = PMOD_CAN_Transmit(&TxMsg_PMOD);
			printf("[PMOD TX %s] ID:0x%03X Data: %02X %02X %02X %02X\r\n",
					(txResult == 1) ? "OK" : "FAIL",
					TxMsg_PMOD.StdId,
					TxMsg_PMOD.Data[0], TxMsg_PMOD.Data[1],
					TxMsg_PMOD.Data[2], TxMsg_PMOD.Data[3]);

			if (txResult != 1)
			{
				printf("[CAN DBG] RF0R=%lu ESR=0x%08lX CH9434_ERR=0x%08lX CH9434_TSR=0x%08lX Mode=0x%02X\r\n",
						MyCAN_GetPendingCount(), MyCAN_GetLastError(),
						PMOD_CAN_GetErrorReg(), PMOD_CAN_GetTxStatusReg(), PMOD_CAN_GetMode());
			}
		}
		
		/* 按键触发发送（可选功能）*/
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			/* 按键1：手动触发发送 - 所有数据都加10保持一致性 */
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
		
		/* 硬件CAN接收（接收来自其他设备的CAN消息）*/
		if (MyCAN_ReceiveFlag())
		{
			MyCAN_Receive(&RxMsg);
			
			/* 串口显示硬件CAN接收到的数据 */
			if (RxMsg.RTR == CAN_RTR_Data)  // 只显示数据帧
			{
				printf("[HW CAN RX] ID:0x%03X DLC:%d Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
						RxMsg.StdId, RxMsg.DLC,
						RxMsg.Data[0], RxMsg.Data[1], RxMsg.Data[2], RxMsg.Data[3],
						RxMsg.Data[4], RxMsg.Data[5], RxMsg.Data[6], RxMsg.Data[7]);
			}
		}
		else if (TimingFlag == 0)
		{
			static uint16_t rxIdleCounter = 0;
			rxIdleCounter++;
			if (rxIdleCounter >= 50000)
			{
				rxIdleCounter = 0;
				printf("[CAN WAIT] RF0R=%lu ESR=0x%08lX\r\n", MyCAN_GetPendingCount(), MyCAN_GetLastError());
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
