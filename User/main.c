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
	char strTx[100];   // 发送数据缓冲区
	char strRx[100];   // 接收数据缓冲区
	uint32_t debug_counter = 0;  // 调试计数器
	
	UART1_Init(115200);  // 初始化串口，波特率115200
	Key_Init();
	MyCAN_Init();       // 初始化STM32硬件CAN接收
	PMOD_CAN_Init();    // 初始化PMOD CAN发送
	Timer_Init();
	
	/* 串口显示初始化信息 */
	UART1_SendString("\r\n========================================\r\n");
	UART1_SendString("PMOD TX / Hardware CAN RX System\r\n");
	UART1_SendString("CAN Mode: Normal (125kbps) ABOM=ON\r\n");
	UART1_SendString("Waiting for PMOD CAN messages...\r\n");
	
	/* 检查CH9434状态 */
	{
		uint8_t mode = PMOD_CAN_GetMode();
		sprintf(strTx, "[CH9434] Mode:0x%02X ", mode);
		UART1_SendString(strTx);
		if(mode == 0x00) {
			UART1_SendString("- Normal OK\r\n");
		} else if(mode == 0x80) {
			UART1_SendString("- Config mode ERROR!\r\n");
		} else if(mode == 0x01) {
			UART1_SendString("- Sleep mode ERROR!\r\n");
		} else {
			UART1_SendString("- Unknown, Check SPI!\r\n");
		}
	}
	UART1_SendString("========================================\r\n");
	
	while (1)
	{
		/* 周期性显示调试信息（每50次循环显示一次CAN状态）*/
		debug_counter++;
		if (debug_counter >= 50000)
		{
			debug_counter = 0;
			uint32_t can_esr = MyCAN_GetLastError();
			uint8_t can_status = MyCAN_GetErrorStatus();
			sprintf(strRx, "[DEBUG] CAN ESR:0x%08lX Status:%d FIFO Pending:%d\r\n",
					can_esr, can_status, CAN_MessagePending(CAN1, CAN_FIFO0));
			UART1_SendString(strRx);
		}

		/* 定时发送（通过PMOD发送CAN）*/
		if (TimingFlag == 1)
		{
			uint8_t txResult;
			TimingFlag = 0;
			
			/* 更新发送数据 */
			TxMsg_PMOD.Data[0]++;
			TxMsg_PMOD.Data[1]++;
			TxMsg_PMOD.Data[2]++;
			TxMsg_PMOD.Data[3]++;
			
			/* 通过PMOD发送 */
			txResult = PMOD_CAN_Transmit(&TxMsg_PMOD);

			/* 串口显示PMOD发送的数据和状态 */
			if(txResult == 1) {
				sprintf(strTx, "[PMOD TX OK] ID:0x%03X Data: %02X %02X %02X %02X\r\n", 
						TxMsg_PMOD.StdId, 
						TxMsg_PMOD.Data[0], 
						TxMsg_PMOD.Data[1], 
						TxMsg_PMOD.Data[2], 
						TxMsg_PMOD.Data[3]);
			} else if(txResult == 0) {
				sprintf(strTx, "[PMOD TX TIMEOUT] ID:0x%03X\r\n", TxMsg_PMOD.StdId);
			} else if(txResult == 2) {
				sprintf(strTx, "[PMOD TX BUSY]\r\n");
			} else {
				sprintf(strTx, "[PMOD TX ERROR] No ACK - Check CAN bus connection!\r\n");
			}
			UART1_SendString(strTx);
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
			
			/* 串口显示按键触发的发送 */
			sprintf(strTx, "[KEY TX] ID:0x%03X Data: %02X %02X %02X %02X\r\n", 
					TxMsg_PMOD.StdId, 
					TxMsg_PMOD.Data[0], 
					TxMsg_PMOD.Data[1], 
					TxMsg_PMOD.Data[2], 
					TxMsg_PMOD.Data[3]);
			UART1_SendString(strTx);
		}
		
		/* 硬件CAN接收（接收来自其他设备的CAN消息）*/
		if (MyCAN_ReceiveFlag())
		{
			MyCAN_Receive(&RxMsg);
			
			/* 串口显示硬件CAN接收到的数据 */
			if (RxMsg.RTR == CAN_RTR_Data)  // 只显示数据帧
			{
				sprintf(strRx, "[HW CAN RX] ID:0x%03X DLC:%d Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 
						RxMsg.StdId, 
						RxMsg.DLC,
						RxMsg.Data[0], 
						RxMsg.Data[1], 
						RxMsg.Data[2], 
						RxMsg.Data[3],
						RxMsg.Data[4], 
						RxMsg.Data[5], 
						RxMsg.Data[6], 
						RxMsg.Data[7]);
				UART1_SendString(strRx);
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
