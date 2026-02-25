#include "stm32f10x.h"                  // Device header

void MyCAN_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	// 开启 AFIO 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// 重映射 CAN1 到 PB8/PB9
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  // 正常模式
	CAN_InitStructure.CAN_Prescaler = 36;		// 波特率 = 36M / 36 / 8 = 125K
	CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;    // 采样点 = (1+5)/8 = 75%
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;  // 启用自动总线恢复
	CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  // Mask=0表示不关心ID的任何位
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;   // 接收所有ID的消息
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

void MyCAN_Transmit(CanTxMsg *TxMessage)
{
	uint8_t TransmitMailbox = CAN_Transmit(CAN1, TxMessage);
	
	uint32_t Timeout = 0;
	while (CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok)
	{
		Timeout ++;
		if (Timeout > 100000)
		{
			break;
		}
	}
}

uint8_t MyCAN_ReceiveFlag(void)
{
	if (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
	{
		return 1;
	}
	return 0;
}

void MyCAN_Receive(CanRxMsg *RxMessage)
{
	CAN_Receive(CAN1, CAN_FIFO0, RxMessage);
}

uint8_t MyCAN_GetErrorStatus(void)
{
	return (uint8_t)((CAN1->ESR >> 4) & 0x07);  // 返回错误状态
}

uint32_t MyCAN_GetLastError(void)
{
	return CAN1->ESR;  // 返回完整的错误状态寄存器
}