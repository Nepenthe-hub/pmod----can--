#include "PMOD_CAN.h"
#include "CH9434.h"
#include "SPI.h"
#include "Delay.h"
#include <string.h>

/* ============================================================
 * 内部状态
 * ============================================================ */
static uint8_t g_pmod_can_ready = 0;

/* 用于诊断读取 IO_SEL_FUN (0x45) 寄存器 */
extern u8_t CH9434RegReadBytes(u8_t reg_ad, u8_t *p_data, u16_t len);
/* 用于直接读 CAN 寄存器 */
extern u32_t CH9434ReadCANReg(u8_t reg_add);

/**
  * @brief  初始化 PMOD CAN 发送模块（使用官方 CH9434D 驱动）
  *
  * 初始化流程参照官方 CAN EVT:
  *   1. BSP_SPI_Init()
  *   2. Delay 500ms 等上电
  *   3. CH9434DefIOFuncEn(6) 启用 CAN 默认引脚
  *   4. CH9434D_CAN_Init() 配置 CAN 参数
  *   5. 软件滤波器设置（接受所有）
  *   6. 启用中断
  *
  * 波特率 = 96MHz / (prescaler * (1 + BS1 + BS2))
  *        = 96MHz / (16 * (1 + 6 + 5))
  *        = 96MHz / (16 * 12)
  *        = 500kbps
  *
  * 如果要 125kbps:
  *        = 96MHz / (64 * 12) = 125kbps  (prescaler=64, BS1=6tq, BS2=5tq)
  */
void PMOD_CAN_Init(void)
{
    CH9434D_CAN_InitTypeDef canInit;
    u8_t result;

    /* 1. SPI 底层已在 main.c 的诊断段初始化过，这里确保一下 */
    BSP_SPI_Init();

    /* 2. 等待 CH9434D 上电就绪（官方 EVT: Delay_Ms(500)） */
    Delay_ms(500);

    /* 3. 启用 CAN 默认引脚 (pin6)
     *    CH9434D 默认使用内部时钟 96MHz，无需启用 HSE(pin5)
     *    无需调用 CH9434InitClkMode()，内部时钟足够 */
    CH9434DefIOFuncEn(CH9434D_DEF_CAN_ADD);

    /* 4. 配置 CAN 参数 — 125kbps, 正常模式 */
    memset(&canInit, 0, sizeof(canInit));
    canInit.CAN_TTCM = DISABLE_T;
    canInit.CAN_ABOM = ENABLE_T;     /* 自动总线恢复 */
    canInit.CAN_AWUM = DISABLE_T;
    canInit.CAN_NART = DISABLE_T;    /* 允许自动重传 */
    canInit.CAN_RFLM = DISABLE_T;
    canInit.CAN_TXFP = DISABLE_T;
    canInit.CAN_Mode = CH9434D_CAN_Mode_Normal;
    canInit.CAN_SJW  = CH9434D_CAN_SJW_tq(1);    /* SJW = 1 tq */
    canInit.CAN_BS1  = CH9434D_CAN_BS1_tq(5);     /* BS1 = 5 tq */
    canInit.CAN_BS2  = CH9434D_CAN_BS2_tq(2);     /* BS2 = 2 tq */
    canInit.CAN_Prescaler = 96;
    /* 波特率 = 96MHz / (96 * (1+5+2)) = 96MHz / 768 = 125kbps */

    result = CH9434D_CAN_Init(&canInit);
    if (result == CH9434D_CAN_InitStatus_Success) {
        g_pmod_can_ready = 1;
    } else {
        g_pmod_can_ready = 0;
        return;
    }

    /* 5. 硬件滤波器 — 接受所有消息（Mask=0） */
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

    /* 6. 启用 FIFO0 接收中断 */
    CH9434D_CAN_ITConfig(CH9434D_CAN_IT_FMP0, ENABLE_T);
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

