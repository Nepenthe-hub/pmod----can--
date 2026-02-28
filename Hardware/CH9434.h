#ifndef __CH9434_H
#define __CH9434_H

#include "stm32f10x.h"

/* ========================================================= */
/* 1. SPI 接口寄存器/命令                                     */
/* ========================================================= */
#define CH9434_SPI_WRITE_BIT        0x80

#define CH9434_REG_IO_SEL_FUN       0x45
#define CH9434_REG_CAN              0x46

/* 默认功能引脚编号 */
#define CH9434_PIN_UART1            1
#define CH9434_PIN_UART2            2
#define CH9434_PIN_UART3            3
#define CH9434_PIN_UART4            4
#define CH9434_PIN_HSE              5
#define CH9434_PIN_CAN              6

/* IO_SEL_FUN 相关命令值 */
#define CH9434_IO_DEF_W_EN          0x03
#define CH9434_IO_CMD_ACT           0xA5
#define CH9434_IO_CMD_COMP          0x5A

/* ========================================================= */
/* 2. CAN 寄存器地址                                           */
/* ========================================================= */
#define CAN_CTLR                    0x00
#define CAN_STATR                   0x01
#define CAN_TSTATR                  0x02
#define CAN_RFIFO0                  0x03
#define CAN_RFIFO1                  0x04
#define CAN_INTENR                  0x05
#define CAN_ERRSR                   0x06
#define CAN_BTIMR                   0x07

#define CAN_TXMIR0                  0x0B
#define CAN_TXMDTR0                 0x0C
#define CAN_TXMDLR0                 0x0D
#define CAN_TXMDHR0                 0x0E

#define CAN_TXMIR1                  0x0F
#define CAN_TXMDTR1                 0x10
#define CAN_TXMDLR1                 0x11
#define CAN_TXMDHR1                 0x12

#define CAN_TXMIR2                  0x13
#define CAN_TXMDTR2                 0x14
#define CAN_TXMDLR2                 0x15
#define CAN_TXMDHR2                 0x16

#define CAN_RXMIR0                  0x17
#define CAN_RXMDTR0                 0x18
#define CAN_RXMDLR0                 0x19
#define CAN_RXMDHR0                 0x1A

#define CAN_RXMIR1                  0x1B
#define CAN_RXMDTR1                 0x1C
#define CAN_RXMDLR1                 0x1D
#define CAN_RXMDHR1                 0x1E

#define CAN_FCTLR                   0x1F
#define CAN_FMCFGR                  0x20
#define CAN_FSCFGR                  0x21
#define CAN_FAFIFOR                 0x22
#define CAN_FWR                     0x23
#define CAN_F0R1                    0x24
#define CAN_F0R2                    0x25

/* ========================================================= */
/* 3. CAN 位定义                                               */
/* ========================================================= */
#define CAN_CTLR_INRQ               (1UL << 0)
#define CAN_CTLR_SLEEP              (1UL << 1)
#define CAN_CTLR_TXFP               (1UL << 2)
#define CAN_CTLR_RFLM               (1UL << 3)
#define CAN_CTLR_NART               (1UL << 4)
#define CAN_CTLR_AWUM               (1UL << 5)
#define CAN_CTLR_ABOM               (1UL << 6)
#define CAN_CTLR_TTCM               (1UL << 7)
#define CAN_CTLR_RST                (1UL << 15)

#define CAN_STATR_INAK              (1UL << 0)
#define CAN_STATR_SLAK              (1UL << 1)

#define CAN_TSTATR_RQCP0            (1UL << 0)
#define CAN_TSTATR_TXOK0            (1UL << 1)
#define CAN_TSTATR_ALST0            (1UL << 2)
#define CAN_TSTATR_TERR0            (1UL << 3)

#define CAN_TSTATR_RQCP1            (1UL << 8)
#define CAN_TSTATR_TXOK1            (1UL << 9)
#define CAN_TSTATR_ALST1            (1UL << 10)
#define CAN_TSTATR_TERR1            (1UL << 11)

#define CAN_TSTATR_RQCP2            (1UL << 16)
#define CAN_TSTATR_TXOK2            (1UL << 17)
#define CAN_TSTATR_ALST2            (1UL << 18)
#define CAN_TSTATR_TERR2            (1UL << 19)

#define CAN_TSTATR_TME0             (1UL << 26)
#define CAN_TSTATR_TME1             (1UL << 27)
#define CAN_TSTATR_TME2             (1UL << 28)

#define CAN_RFIFOx_FMPx             (0xFFUL)
#define CAN_RFIFOx_FULLx            (1UL << 16)
#define CAN_RFIFOx_FOVRx            (1UL << 17)
#define CAN_RFIFOx_RFOMx            (1UL << 18)

#define CAN_TXMIRx_TXRQ             (1UL << 0)
#define CAN_TXMIRx_RTR              (1UL << 1)
#define CAN_TXMIRx_IDE              (1UL << 2)

#define CAN_FCTLR_FINIT             (1UL << 0)

/* ========================================================= */
/* 4. 驱动类型定义                                             */
/* ========================================================= */
typedef struct {
    uint32_t id;        // CAN ID (标准帧11位 或 扩展帧29位)
    uint8_t  dlc;       // 数据长度 (0-8)
    uint8_t  data[8];   // CAN 帧数据
    uint8_t  extended;  // 扩展帧标志 (0 = 标准帧, 1 = 扩展帧)
    uint8_t  rtr;       // 远程帧标志 (0 = 数据帧, 1 = 远程帧)
} CAN_Message;

typedef struct {
    uint8_t  ttcm;
    uint8_t  abom;
    uint8_t  awum;
    uint8_t  nart;
    uint8_t  rflm;
    uint8_t  txfp;
    uint8_t  mode;      // 0:Normal 1:LoopBack 2:Silent 3:SilentLoopBack
    uint8_t  sjw;       // 编码值 0~3 -> 1~4 tq
    uint8_t  bs1;       // 编码值 0~15 -> 1~16 tq
    uint8_t  bs2;       // 编码值 0~7 -> 1~8 tq
    uint16_t prescaler; // 1~1024
} CH9434_CAN_InitTypeDef;

#define CH9434_CAN_MODE_NORMAL          0
#define CH9434_CAN_MODE_LOOPBACK        1
#define CH9434_CAN_MODE_SILENT          2
#define CH9434_CAN_MODE_SILENT_LOOPBACK 3

#define CH9434_CAN_TX_PENDING           0
#define CH9434_CAN_TX_OK                1
#define CH9434_CAN_TX_FAILED            2

/* ========================================================= */
/* 5. 函数声明                                                 */
/* ========================================================= */
void     CH9434_Write8(uint8_t addr, uint8_t val);
uint32_t CH9434_AccessCANReg32(uint8_t can_reg_addr, uint32_t write_val, uint8_t is_write);
void     CH9434_EnableDefaultPin(uint8_t pin_addr);

void     CH9434_CAN_StructInit(CH9434_CAN_InitTypeDef *init);
uint8_t  CH9434_CAN_Init(const CH9434_CAN_InitTypeDef *init);
void     CH9434_CAN_FilterAcceptAll(void);

uint8_t  CH9434_CAN_Transmit(const CAN_Message *msg);
uint8_t  CH9434_CAN_TransmitStatus(uint8_t mailbox);
uint8_t  CH9434_CAN_Receive(CAN_Message *msg, uint8_t fifo);
uint8_t  CH9434_CAN_MessagePending(uint8_t fifo);
void     CH9434_CAN_FIFORelease(uint8_t fifo);

uint8_t  CH9434_CAN_IsInitialized(void);
uint32_t CH9434_CAN_GetErrorReg(void);
uint32_t CH9434_CAN_GetTxStatusReg(void);

void     CH9434_Init_CAN(uint32_t baudrate);
uint8_t  CH9434_SendCANMessage(CAN_Message *msg);
uint8_t  CH9434_ReceiveCANMessage(CAN_Message *msg);
uint8_t  CH9434_GetMode(void);
uint8_t  CH9434_ReadReg(uint8_t can_reg_addr);

#endif /* __CH9434_H */

