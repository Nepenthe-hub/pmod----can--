#include "PMOD_CAN.h"
#include "CH9434.h"
#include "SPI.h"
#include "Delay.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 内部状态
 * ============================================================ */
static uint8_t g_pmod_can_ready = 0;

/* 用于诊断读取 IO_SEL_FUN (0x45) 寄存器 */
extern u8_t CH9434RegReadBytes(u8_t reg_ad, u8_t *p_data, u16_t len);
/* 用于直接读写 CAN 寄存器 */
extern u32_t CH9434ReadCANReg(u8_t reg_add);
extern void  CH9434WriteCANReg(u8_t reg_add, u32_t reg_val);

/**
  * @brief  初始化 PMOD CAN 发送模块（使用官方 CH9434D 驱动）
  *
  * 初始化流程参照官方 CAN EVT:
  *   0. RST# 硬件复位
  *   1. BSP_SPI_Init()
  *   2. INT 引脚配置
  *   3. Delay 500ms 等上电
  *   4. 时钟配置: HSE + PLL -> 96MHz
  *   5. CH9434DefIOFuncEn(6) 启用 CAN 默认引脚
  *   6. CH9434D_CAN_Init() 配置 CAN 参数
  *   7. 滤波器设置（接受所有）
  *   8. 启用中断
  *
  * 波特率 = 96MHz / (96 * (1+5+2)) = 125kbps
  */
/**
  * @brief  驱动 CH9434D RST# 引脚: PB5 推挽输出
  *         低电平复位 -> 高电平释放
  */
static void PMOD_RST_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_5;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &g);

    /* 先拉低 RST# 复位芯片 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
    Delay_ms(50);
    /* 释放复位 */
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
    Delay_ms(50);
}

/**
  * @brief  配置 PB4 为 INT 输入（CH9434D 中断输出，有数据时拉低）
  */
static void PMOD_INT_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    /* PB4 是 JTAG 的 NJTRST，需禁用 JTAG 才能当普通 GPIO */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    g.GPIO_Pin  = GPIO_Pin_4;
    g.GPIO_Mode = GPIO_Mode_IPU;   /* 上拉输入，INT 低有效 */
    GPIO_Init(GPIOB, &g);
}

void PMOD_CAN_Init(void)
{
    CH9434D_CAN_InitTypeDef canInit;
    u8_t result;

    /* 0. 硬件复位 CH9434D — 确保芯片从已知状态启动 */
    PMOD_RST_Init();

    /* 1. SPI 底层初始化 */
    BSP_SPI_Init();

    /* 2. INT 引脚初始化 (PB4，用于后续中断/轮询检测) */
    PMOD_INT_Init();

    /* 3. 等待 CH9434D 上电就绪（官方 EVT: Delay_Ms(500)） */
    Delay_ms(500);

    /* ============================================================
     * 4. 时钟配置 — 参照官方 EVT 的初始化顺序
     *    如果 PMOD 板上有 16MHz 外部晶振，必须启用 HSE + PLL
     *    CH9434D 内部时钟固定 96MHz，但仍写入寄存器让芯片时钟稳定
     * ============================================================ */

    /* 4a. 告知驱动外部晶振频率（如果板上有晶振） */
    CH9434OscXtFreqSet(16000000);

    /* 4b. 启用外部高速振荡器 (HSE) 默认 IO 功能
     *     CH9434D_DEF_HSE_ADD = 5 */
    CH9434DefIOFuncEn(CH9434D_DEF_HSE_ADD);

    /* 4c. 配置时钟模式: xt_en=1(启用外部晶振), freq_mul=1(启用PLL倍频), div=1
     *     最终 CAN 时钟 = 96MHz */
    CH9434InitClkMode(CH9434_ENABLE, CH9434_ENABLE, 1);

    /* 5. 启用 CAN 默认引脚 (CH9434D_DEF_CAN_ADD = 6) */
    CH9434DefIOFuncEn(CH9434D_DEF_CAN_ADD);

    printf("[CH9434] HSE/CLK/CAN IO enabled\r\n");

    /* 6. 配置 CAN 参数 — 125kbps, 正常模式
     *    波特率 = 96MHz / (Prescaler * (1 + BS1 + BS2))
     *           = 96MHz / (96 * (1 + 5 + 2))
     *           = 96MHz / 768 = 125kbps
     *
     *    官方 EVT 参数 (500kbps):
     *      SJW=1, BS1=6, BS2=5, BRP=16 -> 96M/(16*12)=500k
     *
     *    注意: CAN_NART 使用 ENABLE_T (禁止自动重传)
     *          与官方 EVT 一致, 避免总线上无 ACK 时无限重发
     */
    memset(&canInit, 0, sizeof(canInit));
    canInit.CAN_TTCM = DISABLE_T;
    canInit.CAN_ABOM = ENABLE_T;     /* 自动总线恢复 */
    canInit.CAN_AWUM = ENABLE_T;     /* 自动唤醒——防止 CAN 控制器进入 SLEEP */
    canInit.CAN_NART = DISABLE_T;    /* 允许自动重传，提高总线可靠性 */
    canInit.CAN_RFLM = DISABLE_T;
    canInit.CAN_TXFP = DISABLE_T;
    canInit.CAN_Mode = CH9434D_CAN_Mode_Normal;
    canInit.CAN_SJW  = CH9434D_CAN_SJW_tq(1);    /* SJW = 1 tq */
    canInit.CAN_BS1  = CH9434D_CAN_BS1_tq(5);     /* BS1 = 5 tq */
    canInit.CAN_BS2  = CH9434D_CAN_BS2_tq(2);     /* BS2 = 2 tq */
    canInit.CAN_Prescaler = 96;
    /* 波特率 = 96MHz / (96 * (1+5+2)) = 96MHz / 768 = 125kbps */

    result = CH9434D_CAN_Init(&canInit);
    {
        u32_t ctlr_after = CH9434ReadCANReg(CH9434D_CAN_CTLR);
        u32_t statr_after = CH9434ReadCANReg(CH9434D_CAN_STATR);
        printf("[CH9434] CAN_Init result=%d CTLR=0x%08lX STATR=0x%08lX\r\n",
               result, (unsigned long)ctlr_after, (unsigned long)statr_after);
    }
    if (result == CH9434D_CAN_InitStatus_Success) {
        g_pmod_can_ready = 1;
    } else {
        g_pmod_can_ready = 0;
        return;
    }

    /* 7. 硬件滤波器 — 接受所有消息（Mask=0） */
    {
        CH9434D_CAN_FilterInitTypeDef filterInit;
        memset(&filterInit, 0, sizeof(filterInit));
        filterInit.CAN_FilterNumber         = 0;
        filterInit.CAN_FilterMode           = CH9434D_CAN_FilterMode_IdMask;
        filterInit.CAN_FilterScale          = CH9434D_CAN_FilterScale_32bit;
        filterInit.CAN_FilterIdHigh         = 0x0000;
        filterInit.CAN_FilterIdLow          = 0x0000;
        filterInit.CAN_FilterMaskIdHigh     = 0x0000;  /* Mask=0 接受所有 */
        filterInit.CAN_FilterMaskIdLow      = 0x0000;
        filterInit.CAN_FilterFIFOAssignment = CH9434D_CAN_Filter_FIFO0;
        filterInit.CAN_FilterActivation     = ENABLE_T;
        CH9434D_CAN_FilterInit(&filterInit);
    }

    /* 8. 启用 FIFO0 接收中断 */
    CH9434D_CAN_ITConfig(CH9434D_CAN_IT_FMP0, ENABLE_T);

    /* 9. 确保 CAN 控制器已唤醒（初始化后可能自动进入 SLEEP） */
    {
        u32_t ctlr_chk = CH9434ReadCANReg(CH9434D_CAN_CTLR);
        if (ctlr_chk & CH9434D_CAN_CTLR_SLEEP) {
            u8_t wk = CH9434D_CAN_WakeUp();
            printf("[CH9434] CAN was in SLEEP, WakeUp=%d\r\n", wk);
        }
        ctlr_chk = CH9434ReadCANReg(CH9434D_CAN_CTLR);
        printf("[CH9434] Final CTLR=0x%02lX (expect 0x60: ABOM+AWUM, no SLEEP)\r\n",
               (unsigned long)(ctlr_chk & 0xFF));
    }
}

/**
  * @brief  通过 PMOD 发送 CAN 消息
  * @param  TxMessage: STM32 格式的 CAN 发送消息
  * @retval 1: 发送成功, 0: 发送失败/超时
  */
uint8_t PMOD_CAN_Transmit(CanTxMsg *TxMessage)
{
    CH9434D_CanTxMsg txMsg;
    u8_t mailbox;
    u16_t timeout = 0;
    uint8_t i;

    if (!g_pmod_can_ready || TxMessage == 0) {
        return 0;
    }

    /* 确保 CAN 控制器不在睡眠模式（解决 Mode=0x5A / SLEEP位置位问题） */
    {
        u32_t ctlr = CH9434ReadCANReg(CH9434D_CAN_CTLR);
        if (ctlr & CH9434D_CAN_CTLR_SLEEP) {
            CH9434D_CAN_WakeUp();
        }
    }

    /* 转换: STM32 CanTxMsg -> CH9434D_CanTxMsg */
    if (TxMessage->IDE == CAN_Id_Extended) {
        txMsg.ExtId = TxMessage->ExtId;
        txMsg.StdId = 0;
        txMsg.IDE = CH9434D_CAN_Id_Extended;
    } else {
        txMsg.StdId = TxMessage->StdId;
        txMsg.ExtId = 0;
        txMsg.IDE = CH9434D_CAN_Id_Standard;
    }

    txMsg.RTR = (TxMessage->RTR == CAN_RTR_Remote) ? CH9434D_CAN_RTR_Remote : CH9434D_CAN_RTR_Data;
    txMsg.DLC = TxMessage->DLC;
    for (i = 0; i < txMsg.DLC && i < 8; i++) {
        txMsg.Data[i] = TxMessage->Data[i];
    }

    /* 发送 */
    mailbox = CH9434D_CAN_Transmit(&txMsg);
    if (mailbox == CH9434D_CAN_TxStatus_NoMailBox) {
        return 0;
    }

    /* 等待发送完成 */
    while ((CH9434D_CAN_TransmitStatus(mailbox) != CH9434D_CAN_TxStatus_Ok) && (timeout < 0xFFF)) {
        timeout++;
    }

    return (timeout < 0xFFF) ? 1 : 0;
}

/**
  * @brief  通过 PMOD 读取 CAN 接收消息
  * @param  RxMessage: STM32 格式的 CAN 接收消息
  * @retval 1: 成功接收, 0: 无消息
  */
uint8_t PMOD_CAN_Receive(CanRxMsg *RxMessage)
{
    CH9434D_CanRxMsg rxMsg;
    uint8_t i;

    /* 检查 FIFO0 是否有消息 */
    if (CH9434D_CAN_MessagePending(CH9434D_CAN_FIFO0) == 0) {
        return 0;
    }

    /* 接收 */
    CH9434D_CAN_Receive(CH9434D_CAN_FIFO0, &rxMsg);

    /* 转换: CH9434D_CanRxMsg -> STM32 CanRxMsg */
    if (rxMsg.IDE == CH9434D_CAN_Id_Extended) {
        RxMessage->IDE   = CAN_Id_Extended;
        RxMessage->ExtId = rxMsg.ExtId;
        RxMessage->StdId = 0;
    } else {
        RxMessage->IDE   = CAN_Id_Standard;
        RxMessage->StdId = rxMsg.StdId;
        RxMessage->ExtId = 0;
    }

    RxMessage->RTR = (rxMsg.RTR == CH9434D_CAN_RTR_Remote) ? CAN_RTR_Remote : CAN_RTR_Data;
    RxMessage->DLC = rxMsg.DLC;
    for (i = 0; i < rxMsg.DLC && i < 8; i++) {
        RxMessage->Data[i] = rxMsg.Data[i];
    }
    RxMessage->FMI = rxMsg.FMI;

    return 1;
}

/**
  * @brief  获取 CH9434D CAN 模式字节
  */
uint8_t PMOD_CAN_GetMode(void)
{
    u32_t ctlr = CH9434ReadCANReg(CH9434D_CAN_CTLR);
    return (uint8_t)(ctlr & 0xFF);
}

/**
  * @brief  获取 CH9434D CAN 错误寄存器
  */
uint32_t PMOD_CAN_GetErrorReg(void)
{
    return (uint32_t)CH9434ReadCANReg(CH9434D_CAN_ERRSR);
}

/**
  * @brief  获取 CH9434D CAN 发送状态寄存器
  */
uint32_t PMOD_CAN_GetTxStatusReg(void)
{
    return (uint32_t)CH9434ReadCANReg(CH9434D_CAN_TSTATR);
}

/**
  * @brief  CH9434 CAN 是否初始化成功
  */
uint8_t PMOD_CAN_IsReady(void)
{
    return g_pmod_can_ready;
}

/**
  * @brief  读取 IO_SEL_FUN (0x45) 原始 4 字节（诊断用）
  */
uint8_t PMOD_CAN_ReadIOFuncRaw(uint8_t out4[4])
{
    CH9434RegReadBytes(CH9434_IO_SEL_FUN_CFG, (u8_t *)out4, 4);
    return 1;
}

/**
  * @brief  CH9434D CAN Loopback 自测
  *         临时切入 Loopback 模式，发一帧、收一帧，验证 CAN 控制器内部正常
  * @retval 1: 自测通过, 0: 失败
  */
uint8_t PMOD_CAN_LoopbackTest(void)
{
    CH9434D_CAN_InitTypeDef canInit;
    CH9434D_CanTxMsg txMsg;
    CH9434D_CanRxMsg rxMsg;
    u8_t mailbox, result;
    u16_t timeout;

    if (!g_pmod_can_ready) return 0;

    /* 1. 切入 Loopback 模式（需要重新初始化） */
    memset(&canInit, 0, sizeof(canInit));
    canInit.CAN_TTCM = DISABLE_T;
    canInit.CAN_ABOM = ENABLE_T;
    canInit.CAN_AWUM = ENABLE_T;
    canInit.CAN_NART = DISABLE_T;
    canInit.CAN_RFLM = DISABLE_T;
    canInit.CAN_TXFP = DISABLE_T;
    canInit.CAN_Mode = CH9434D_CAN_Mode_LoopBack;  /* <<< Loopback */
    canInit.CAN_SJW  = CH9434D_CAN_SJW_tq(1);
    canInit.CAN_BS1  = CH9434D_CAN_BS1_tq(5);
    canInit.CAN_BS2  = CH9434D_CAN_BS2_tq(2);
    canInit.CAN_Prescaler = 96;

    result = CH9434D_CAN_Init(&canInit);
    if (result != CH9434D_CAN_InitStatus_Success) {
        printf("[LB TEST] CAN_Init(Loopback) FAIL\r\n");
        return 0;
    }

    /* 重新配置滤波器 */
    {
        CH9434D_CAN_FilterInitTypeDef filt;
        memset(&filt, 0, sizeof(filt));
        filt.CAN_FilterNumber         = 0;
        filt.CAN_FilterMode           = CH9434D_CAN_FilterMode_IdMask;
        filt.CAN_FilterScale          = CH9434D_CAN_FilterScale_32bit;
        filt.CAN_FilterIdHigh         = 0x0000;
        filt.CAN_FilterIdLow          = 0x0000;
        filt.CAN_FilterMaskIdHigh     = 0x0000;
        filt.CAN_FilterMaskIdLow      = 0x0000;
        filt.CAN_FilterFIFOAssignment = CH9434D_CAN_Filter_FIFO0;
        filt.CAN_FilterActivation     = ENABLE_T;
        CH9434D_CAN_FilterInit(&filt);
    }

    /* 2. 发一帧测试消息 */
    memset(&txMsg, 0, sizeof(txMsg));
    txMsg.StdId = 0x7FF;
    txMsg.IDE   = CH9434D_CAN_Id_Standard;
    txMsg.RTR   = CH9434D_CAN_RTR_Data;
    txMsg.DLC   = 4;
    txMsg.Data[0] = 0xDE;
    txMsg.Data[1] = 0xAD;
    txMsg.Data[2] = 0xBE;
    txMsg.Data[3] = 0xEF;

    mailbox = CH9434D_CAN_Transmit(&txMsg);
    if (mailbox == CH9434D_CAN_TxStatus_NoMailBox) {
        printf("[LB TEST] No mailbox\r\n");
        return 0;
    }

    /* 等待发送完成 */
    timeout = 0;
    while ((CH9434D_CAN_TransmitStatus(mailbox) != CH9434D_CAN_TxStatus_Ok) && (timeout < 0xFFFF)) {
        timeout++;
    }
    if (timeout >= 0xFFFF) {
        printf("[LB TEST] TX timeout\r\n");
        return 0;
    }

    /* 3. 等待接收（Loopback 下会自动回到 FIFO0） */
    timeout = 0;
    while ((CH9434D_CAN_MessagePending(CH9434D_CAN_FIFO0) == 0) && (timeout < 0xFFFF)) {
        timeout++;
    }
    if (timeout >= 0xFFFF) {
        printf("[LB TEST] RX timeout (no loopback message)\r\n");
        return 0;
    }

    /* 4. 读取并验证 */
    CH9434D_CAN_Receive(CH9434D_CAN_FIFO0, &rxMsg);

    result = (rxMsg.StdId == 0x7FF &&
              rxMsg.DLC == 4 &&
              rxMsg.Data[0] == 0xDE &&
              rxMsg.Data[1] == 0xAD &&
              rxMsg.Data[2] == 0xBE &&
              rxMsg.Data[3] == 0xEF) ? 1 : 0;

    printf("[LB TEST] TX->RX: ID=0x%03lX DLC=%d Data=%02X %02X %02X %02X => %s\r\n",
           (unsigned long)rxMsg.StdId, rxMsg.DLC,
           rxMsg.Data[0], rxMsg.Data[1], rxMsg.Data[2], rxMsg.Data[3],
           result ? "PASS" : "DATA MISMATCH");

    return result;
}

