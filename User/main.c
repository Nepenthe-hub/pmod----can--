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

/*============================================================================
 *  硬件级 SPI 诊断 — 逐步排查 CH9434D 不响应的原因
 *  测试内容:
 *    1. MISO 是否悬空（内部下拉测试）
 *    2. 手动 bit-bang 读寄存器（排除驱动 bug）
 *    3. MOSI/MISO 交换测试
 *    4. 遍历 PB12~PB15 全排列测试（排除引脚接错）
 *    5. 检查 CH9434D INT 引脚
 *============================================================================*/
static uint8_t manual_spi_xfer(GPIO_TypeDef *port,
    uint16_t pin_sck, uint16_t pin_mosi, uint16_t pin_miso,
    uint8_t tx)
{
    uint8_t rx = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        GPIO_ResetBits(port, pin_sck);
        if (tx & 0x80) GPIO_SetBits(port, pin_mosi);
        else GPIO_ResetBits(port, pin_mosi);
        Delay_us(5);
        GPIO_SetBits(port, pin_sck);
        Delay_us(5);
        rx <<= 1;
        if (GPIO_ReadInputDataBit(port, pin_miso)) rx |= 1;
        tx <<= 1;
    }
    GPIO_ResetBits(port, pin_sck);
    return rx;
}

static void run_hw_diagnostic(void)
{
    GPIO_InitTypeDef g;
    uint8_t val;
    
    printf("\r\n");
    printf("######################################################\r\n");
    printf("#           CH9434D SPI 硬件级底层诊断                  #\r\n");
    printf("######################################################\r\n");
    printf("当前配置: CS=PB3, SCK=PB13, MISO=PB14, MOSI=PB15\r\n\r\n");
    
    /* 开启 GPIOB + AFIO 时钟，禁用 JTAG 释放 PB4 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* ---------- 测试 1: CS(PB3) 引脚检测 ---------- */
    printf("=== TEST 1: CS(PB3) 引脚检测 ===\r\n");
    g.GPIO_Pin = GPIO_Pin_3;
    g.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &g);
    Delay_us(100);
    val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
    printf("  PB3(下拉) = %d %s\r\n", val, val ? "<-- 有外部驱动" : "(悬空)");
    g.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &g);
    Delay_us(100);
    val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
    printf("  PB3(上拉) = %d\r\n\r\n", val);

    /* ---------- 测试 2: MISO(PB14) 悬空/驱动检测 ---------- */
    printf("=== TEST 2: MISO(PB14) 悬空/驱动检测 ===\r\n");
    g.GPIO_Pin = GPIO_Pin_14;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &g);
    Delay_us(10);
    val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
    printf("  浮空输入: MISO = %d\r\n", val);
    g.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &g);
    Delay_us(100);
    val = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
    printf("  下拉输入: MISO = %d %s\r\n\r\n", val,
           val ? "<-- 有外部信号拉高" : "<-- 悬空!!");

    /* ---------- 测试 3: 关键引脚电平汇总 ---------- */
    printf("=== TEST 3: 关键引脚下拉检测 ===\r\n");
    {
        uint16_t test_pins[] = {GPIO_Pin_3, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};
        const char *test_names[] = {"PB3(CS)", "PB13(SCK)", "PB14(MISO)", "PB15(MOSI)"};
        uint8_t i;
        for (i = 0; i < 4; i++) {
            g.GPIO_Pin = test_pins[i];
            g.GPIO_Mode = GPIO_Mode_IPD;
            GPIO_Init(GPIOB, &g);
        }
        Delay_us(100);
        for (i = 0; i < 4; i++) {
            val = GPIO_ReadInputDataBit(GPIOB, test_pins[i]);
            printf("  %s(下拉) = %d %s\r\n", test_names[i], val,
                   val ? "<-- 有外部驱动" : "(悬空)");
        }
    }
    printf("\r\n");

    /* ---------- 测试 4: 正常配置 手动读寄存器 ---------- */
    printf("=== TEST 4: 手动 bit-bang 读寄存器 (CS=PB4) ===\r\n");
    BSP_SPI_Init();   /* 初始化 SPI (含 PB4 CS + JTAG禁用) */
    Delay_us(10);
    
    /* 读 reg 0x00 */
    SPI_CS_LOW();
    Delay_us(10);
    val = manual_spi_xfer(GPIOB, GPIO_Pin_13, GPIO_Pin_15, GPIO_Pin_14, 0x00);
    printf("  addr phase resp: 0x%02X\r\n", val);
    Delay_us(30);
    val = manual_spi_xfer(GPIOB, GPIO_Pin_13, GPIO_Pin_15, GPIO_Pin_14, 0xFF);
    printf("  data phase resp: 0x%02X\r\n", val);
    SPI_CS_HIGH();
    
    /* 读 0x48 (CLK_CTRL_CFG) */
    Delay_us(10);
    SPI_CS_LOW();
    Delay_us(10);
    manual_spi_xfer(GPIOB, GPIO_Pin_13, GPIO_Pin_15, GPIO_Pin_14, 0x48);
    Delay_us(30);
    val = manual_spi_xfer(GPIOB, GPIO_Pin_13, GPIO_Pin_15, GPIO_Pin_14, 0xFF);
    SPI_CS_HIGH();
    printf("  reg 0x48 (CLK_CTRL): 0x%02X\r\n\r\n", val);

    /* ---------- 测试 5: 交换 MOSI/MISO 测试 ---------- */
    printf("=== TEST 5: 交换 MOSI(PB14)/MISO(PB15) 测试 ===\r\n");
    g.GPIO_Pin = GPIO_Pin_14;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &g);
    g.GPIO_Pin = GPIO_Pin_15;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &g);
    
    SPI_CS_HIGH();
    Delay_us(10);
    SPI_CS_LOW();
    Delay_us(10);
    manual_spi_xfer(GPIOB, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15, 0x48);
    Delay_us(30);
    val = manual_spi_xfer(GPIOB, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15, 0xFF);
    SPI_CS_HIGH();
    printf("  SWAPPED read reg 0x48: 0x%02X\r\n\r\n", val);

    /* ---------- 测试 6: SCK/MOSI/MISO 六种排列 (CS固定PB4) ---------- */
    printf("=== TEST 6: PB13/14/15 排列测试 (CS固定PB4) ===\r\n");
    printf("  格式: [SCK MOSI MISO] => reg0x48读值\r\n");
    {
        /* PB13/14/15 的 6 种排列 */
        uint16_t combos[][3] = {
            {GPIO_Pin_13, GPIO_Pin_15, GPIO_Pin_14}, /* 正常 */
            {GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15}, /* MOSI/MISO交换 */
            {GPIO_Pin_14, GPIO_Pin_13, GPIO_Pin_15},
            {GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_13},
            {GPIO_Pin_15, GPIO_Pin_13, GPIO_Pin_14},
            {GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13},
        };
        const char *combo_names[] = {
            "PB13 PB15 PB14",
            "PB13 PB14 PB15",
            "PB14 PB13 PB15",
            "PB14 PB15 PB13",
            "PB15 PB13 PB14",
            "PB15 PB14 PB13",
        };
        uint8_t found_any = 0;
        uint8_t ci;
        for (ci = 0; ci < 6; ci++) {
            uint16_t p_sck  = combos[ci][0];
            uint16_t p_mosi = combos[ci][1];
            uint16_t p_miso = combos[ci][2];
            
            g.GPIO_Pin = p_sck | p_mosi;
            g.GPIO_Mode = GPIO_Mode_Out_PP;
            g.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &g);
            g.GPIO_Pin = p_miso;
            g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOB, &g);
            
            GPIO_ResetBits(GPIOB, p_sck);
            SPI_CS_HIGH();
            Delay_us(10);
            SPI_CS_LOW();
            Delay_us(10);
            manual_spi_xfer(GPIOB, p_sck, p_mosi, p_miso, 0x48);
            Delay_us(30);
            val = manual_spi_xfer(GPIOB, p_sck, p_mosi, p_miso, 0xFF);
            SPI_CS_HIGH();
            Delay_us(5);
            
            printf("  [%s] => 0x%02X %s\r\n", combo_names[ci], val,
                   (val != 0xFF) ? "<<<< 响应!" : "");
            if (val != 0xFF) found_any = 1;
        }
        
        if (!found_any) {
            printf("  !! 6种排列全部0xFF !! 检查:\r\n");
            printf("     1. CH9434D 供电/GND\r\n");
            printf("     2. RST 引脚是否释放\r\n");
            printf("     3. 外部晶振是否起振\r\n");
        }
    }
    printf("\r\n");

    /* 恢复正常 SPI */
    BSP_SPI_Init();
    
    printf("######################################################\r\n");
    printf("#  诊断完成 — CS=PB3, SCK=PB13, MISO=PB14, MOSI=PB15 #\r\n");
    printf("#  如果TEST4读到非0xFF: SPI通信正常!                    #\r\n");
    printf("######################################################\r\n\r\n");
}

int main(void)
{
	UART1_Init(115200);
	printf("\r\n[BOOT] UART OK\r\n");

	Key_Init();
	printf("[BOOT] Key OK\r\n");

	/* ========== 硬件级诊断 ========== */
	run_hw_diagnostic();

	MyCAN_Init();       // 初始化STM32硬件CAN接收
	printf("[BOOT] MyCAN OK\r\n");

	/* 初始化 PMOD CAN（内部会：SPI初始化 -> 500ms延时 -> 启用CAN引脚 -> CAN_Init） */
	printf("[BOOT] Init CH9434D CAN (official driver)...\r\n");
	PMOD_CAN_Init();
	printf("[BOOT] PMOD Init %s\r\n", PMOD_CAN_IsReady() ? "OK" : "FAIL");

	Timer_Init();
	printf("[BOOT] Timer OK\r\n");

	/* 串口显示初始化信息 */
	{
		uint8_t ioRaw[4] = {0};
		printf("========================================\r\n");
		printf("PMOD TX / Hardware CAN RX System\r\n");
		printf("  CH9434D Official Driver Ported\r\n");
		printf("========================================\r\n");
		printf("[CAN MAP] %s\r\n", MYCAN_USE_REMAP_PB8_PB9 ? "PB8/PB9" : "PA11/PA12");
		PMOD_CAN_ReadIOFuncRaw(ioRaw);
		printf("[CH9434 IO_SEL] 45h: %02X %02X %02X %02X\r\n", ioRaw[0], ioRaw[1], ioRaw[2], ioRaw[3]);
		printf("[CH9434] Ready=%d Mode=0x%02X\r\n", PMOD_CAN_IsReady(), PMOD_CAN_GetMode());
	}
	
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
				uint8_t ioRawDbg[4] = {0};
				PMOD_CAN_ReadIOFuncRaw(ioRawDbg);
				printf("[CAN DBG] RF0R=%lu ESR=0x%08lX CH9434_ERR=0x%08lX CH9434_TSR=0x%08lX Mode=0x%02X\r\n",
						MyCAN_GetPendingCount(), MyCAN_GetLastError(),
						PMOD_CAN_GetErrorReg(), PMOD_CAN_GetTxStatusReg(), PMOD_CAN_GetMode());
				printf("[SPI RAW] 45h: %02X %02X %02X %02X\r\n",
						ioRawDbg[0], ioRawDbg[1], ioRawDbg[2], ioRawDbg[3]);
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
