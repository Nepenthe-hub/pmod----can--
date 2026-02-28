/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH9434.H
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : SPI to Serial Port Chip CH9434 Operating Interface
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#ifndef __CH9434_H
#define __CH9434_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                              Variable type definition
 * -----------------------------------------------------------------------------
 */
#ifndef u8_t
typedef unsigned char u8_t;
#endif
#ifndef u16_t
typedef unsigned short u16_t;
#endif
#ifndef u32_t
typedef unsigned long u32_t;
#endif

/* -----------------------------------------------------------------------------
 *                             Chip model definition
 * -----------------------------------------------------------------------------
 */
#define CHIP_MODEL_CH9434A 0
#define CHIP_MODEL_CH9434D 1
#define USE_CHIP_MODEL CHIP_MODEL_CH9434D

/* -----------------------------------------------------------------------------
 *                          Interface definition defines
 * -----------------------------------------------------------------------------
 */
#define CH9434D_IF_SPI 0
#define CH9434D_IF_IIC 1
#define CH9434D_IF_SEL CH9434D_IF_SPI

/* -----------------------------------------------------------------------------
 *               Call external interface function - User layer definition
 * -----------------------------------------------------------------------------
 */

extern void CH9434_US_DELAY (void);
extern void CH9434_SPI_SCS_OP (u8_t dat);
extern u8_t CH9434_SPI_WRITE_BYTE (u8_t dat);

#if ((USE_CHIP_MODEL == CHIP_MODEL_CH9434D) && (CH9434D_IF_SEL == CH9434D_IF_IIC))
extern void CH9434_IIC_START (void);
extern void CH9434_IIC_SEND_ADD (u8_t dir);
extern void CH9434_IIC_SEND_DATA (u8_t dat);
extern u8_t CH9434_IIC_READ_DATA (void);
extern void CH9434_IIC_STOP (void);
#endif

/* -----------------------------------------------------------------------------
 *                              Macro definition
 * -----------------------------------------------------------------------------
 */
/* Enable Bit */
#define CH9434_ENABLE 1
#define CH9434_DISABLE 0

/* Serial Port Number */
#define CH9434_UART_IDX_0 0
#define CH9434_UART_IDX_1 1
#define CH9434_UART_IDX_2 2
#define CH9434_UART_IDX_3 3

/* Register Operations */
#define CH9434_REG_OP_WRITE 0x80
#define CH9434_REG_OP_READ 0x00

/* Register */
#define CH9434_UART0_REG_OFFSET_ADD 0x00
#define CH9434_UART1_REG_OFFSET_ADD 0x10
#define CH9434_UART2_REG_OFFSET_ADD 0x20
#define CH9434_UART3_REG_OFFSET_ADD 0x30
#define CH9434_UARTx_RBR_ADD 0
#define CH9434_UARTx_THR_ADD 0
#define CH9434_UARTx_IER_ADD 1
#define CH9434_UARTx_IIR_ADD 2
#define CH9434_UARTx_FCR_ADD 2
#define CH9434_UARTx_LCR_ADD 3
#define CH9434_UARTx_BIT_DLAB (1 << 7)
#define CH9434_UARTx_MCR_ADD 4
#define CH9434_UARTx_LSR_ADD 5
#define CH9434_UARTx_MSR_ADD 6
#define CH9434_UARTx_SCR_ADD 7
#define CH9434_UARTx_DLL_ADD 0
#define CH9434_UARTx_DLM_ADD 1

// TNOW UART-related
#define CH9434_TNOW_CTRL_CFG_ADD 0x41
#define CH9434_BIT_TNOW_OUT_POLAR 0xF0
#define CH9434_BIT_TNOW_OUT_EN 0x0F
#define CH9434_FIFO_CTRL_ADD 0x42
#define CH9434_FIFO_CTRL_TR (1 << 4)
#define CH9434_FIFO_UART_IDX 0x0F
#define CH9434_FIFO_CTRL_L_ADD 0x43
#define CH9434_FIFO_CTRL_H_ADD 0x44
#define CH9434_IO_SEL_FUN_CFG 0x45
#define CH9434D_CAN_REG 0x46

// Power clock settings
#define CH9434_CLK_CTRL_CFG_ADD 0x48
#define CH9434_CLK_CTRL_MOD (3 << 6)
#define CH9434_XT_POWER_EN (1 << 5)
#define CH9434_CLK_DIV_MASK 0x1F
#define CH9434_SLEEP_MOD_CFG_ADD 0x4A

// GPIO-related settings // Enable GPIO functions
#define CH9434_GPIO_FUNC_EN_0 0x50
#define CH9434_GPIO_FUNC_EN_1 0x51
#define CH9434_GPIO_FUNC_EN_2 0x52
#define CH9434_GPIO_FUNC_EN_3 0x53
// GPIO direction selection - CH9434D IO mode configuration
#define CH9434_GPIO_DIR_MOD_0 0x54
#define CH9434_GPIO_DIR_MOD_1 0x55
#define CH9434_GPIO_DIR_MOD_2 0x56
#define CH9434_GPIO_DIR_MOD_3 0x57
// GPIO Pull-Up Configuration
#define CH9434_GPIO_PU_MOD_0 0x58
#define CH9434_GPIO_DIR_MOD_4 0x58
#define CH9434_GPIO_PU_MOD_1 0x59
#define CH9434_GPIO_DIR_MOD_5 0x59
#define CH9434_GPIO_PU_MOD_2 0x5A
#define CH9434_GPIO_DIR_MOD_6 0x5A
#define CH9434_GPIO_PU_MOD_3 0x5B
#define CH9434_GPIO_DIR_MOD_7 0x5B
// GPIO pull-down setting - CH9434D IO setting for control of IO pin state
#define CH9434_GPIO_PD_MOD_0 0x5C
#define CH9434_GPIO_SET_0 0x5C
#define CH9434_GPIO_PD_MOD_1 0x5D
#define CH9434_GPIO_SET_1 0x5D
#define CH9434_GPIO_PD_MOD_2 0x5E
#define CH9434_GPIO_RESET_0 0x5E
#define CH9434_GPIO_PD_MOD_3 0x5F
#define CH9434_GPIO_RESET_1 0x5F
// GPIO pin level - CH9434D IO reading and pull-up/pull-down settings
#define CH9434_GPIO_PIN_VAL_0 0x60
#define CH9434_GPIO_PIN_VAL_1 0x61
#define CH9434_GPIO_PIN_VAL_2 0x62
#define CH9434_GPIO_PIN_VAL_3 0x63

/* Serial Port Parameter Definition */
/* FIFO Size */
#define CH9434_UART_FIFO_MODE_256 0   // 256
#define CH9434_UART_FIFO_MODE_512 1   // 512
#define CH9434_UART_FIFO_MODE_1024 2  // 1024
#define CH9434_UART_FIFO_MODE_1280 3  // 1280
/* Character Size */
#define CH9434_UART_5_BITS_PER_CHAR 5
#define CH9434_UART_6_BITS_PER_CHAR 6
#define CH9434_UART_7_BITS_PER_CHAR 7
#define CH9434_UART_8_BITS_PER_CHAR 8
/* Stop Bits */
#define CH9434_UART_ONE_STOP_BIT 1
#define CH9434_UART_TWO_STOP_BITS 2
/* Parity settings */
#define CH9434_UART_NO_PARITY 0x00
#define CH9434_UART_ODD_PARITY 0x01
#define CH9434_UART_EVEN_PARITY 0x02
#define CH9434_UART_MARK_PARITY 0x03
#define CH9434_UART_SPACE_PARITY 0x04

/* Current Sequence Number */
#define CH9434_TNOW_POLAR_NORMAL 0  // Normal output
#define CH9434_TNOW_POLAR_OPPO 1    // Reverse output

#define CH9434_TNOW_0 0
#define CH9434_TNOW_1 1
#define CH9434_TNOW_2 2
#define CH9434_TNOW_3 3

/* Low Power Mode */
#define CH9434_LOWPOWER_INVALID 0
#define CH9434_LOWPOWER_SLEEP 1

/* GPIO Number */
#define CH9434_GPIO_0 0
#define CH9434_GPIO_1 1
#define CH9434_GPIO_2 2
#define CH9434_GPIO_3 3
#define CH9434_GPIO_4 4
#define CH9434_GPIO_5 5
#define CH9434_GPIO_6 6
#define CH9434_GPIO_7 7
#define CH9434_GPIO_8 8
#define CH9434_GPIO_9 9
#define CH9434_GPIO_10 10
#define CH9434_GPIO_11 11
#define CH9434_GPIO_12 12
#define CH9434_GPIO_13 13
#define CH9434_GPIO_14 14
#define CH9434_GPIO_15 15
#define CH9434_GPIO_16 16
#define CH9434_GPIO_17 17
#define CH9434_GPIO_18 18
#define CH9434_GPIO_19 19
#define CH9434_GPIO_20 20
#define CH9434_GPIO_21 21
#define CH9434_GPIO_22 22
#define CH9434_GPIO_23 23
#define CH9434_GPIO_24 24

/* GPIO Enable */
#define CH9434_GPIO_ENABLE 1
#define CH9434_GPIO_DISABLE 0

/* GPIO Direction Setting */
#define CH9434_GPIO_DIR_IN 0
#define CH9434_GPIO_DIR_OUT 1

/* GPIO Pull-Up Enable */
#define CH9434_GPIO_PU_ENABLE 1
#define CH9434_GPIO_PU_DISABLE 0

/* GPIO pull-up enable */
#define CH9434_GPIO_PD_ENABLE 1
#define CH9434_GPIO_PD_DISABLE 0

/* GPIO Output Configuration */
#define CH9434_GPIO_SET 1
#define CH9434_GPIO_RESET 0

/* CH9434D IO mode configuration */
#define CH9434_GPIO_Mode_IN_FLOATING 1
#define CH9434_GPIO_Mode_IPD_OR_IPU 2
#define CH9434_GPIO_Mode_Out_PP 0
#define CH9434_GPIO_Mode_Out_OD 1

#define CH9434_GPIO_Mode_IN 0
#define CH9434_GPIO_Mode_OUT 3

#define CH9434_GPIO_SET_PU 1
#define CH9434_GPIO_SET_PD 0


/* -----------------------------------------------------------------------------
 *                      CH9434D IIC/SPI Macro Definitions
 * -----------------------------------------------------------------------------
 */


/* IIC address retention bit */
#define CH9434D_IIC_ADD_DEF 0x28

/* IIC transmission direction */
#define CH9434D_IIC_DIR_W 0
#define CH9434D_IIC_DIR_R 1

/* -----------------------------------------------------------------------------
 *                       CH9434D I/O Multiplexing Selection
 * -----------------------------------------------------------------------------
 */
// IO-related commands
#define CH9434D_IO_MULTI_W_EN 0x01
#define CH9434D_IO_MULTI_R_EN 0x81
#define CH9434D_IO_MULTI_W_ID 0x02
#define CH9434D_IO_MULTI_R_ID 0x82
#define CH9434D_IO_DEF_W_EN 0x03
#define CH9434D_IO_DEF_R_EN 0x83

// Default IO setting address
#define CH9434D_DEF_U1_ADD 1
#define CH9434D_DEF_U2_ADD 2
#define CH9434D_DEF_U3_ADD 3
#define CH9434D_DEF_U4_ADD 4
#define CH9434D_DEF_HSE_ADD 5
#define CH9434D_DEF_CAN_ADD 6
// #define CH9434D_DEF_RES_ADD             6
#define CH9434D_DEF_CTS1_ADD 8
#define CH9434D_DEF_CTS4_ADD 9

// Reuse the IO setting address
#define CH9434D_MUL_INT_ADD 1
#define CH9434D_MUL_TNOW1_ADD 2
#define CH9434D_MUL_TNOW2_ADD 3
#define CH9434D_MUL_TNOW3_ADD 4
#define CH9434D_MUL_TNOW4_ADD 5
// #define CH9434D_MUL_CTS2_ADD            6
// #define CH9434D_MUL_CTS3_ADD            7
#define CH9434D_MUL_RTS1_ADD 8
// #define CH9434D_MUL_RTS2_ADD            9
// #define CH9434D_MUL_RTS3_ADD            10
#define CH9434D_MUL_RTS4_ADD 11
#define CH9434D_MUL_GPIO0_ADD 12
#define CH9434D_MUL_GPIO1_ADD 13
#define CH9434D_MUL_GPIO2_ADD 14
#define CH9434D_MUL_GPIO3_ADD 15
// #define CH9434D_MUL_GPIO4_ADD           16

// Command Control and Response of I/O
#define CH9434D_IO_CMD_ACT 0xA5
#define CH9434D_IO_CMD_COMP 0x5A


/* -----------------------------------------------------------------------------
 *                                   CH9434D CAN Function
 * -----------------------------------------------------------------------------
 */
/*******************  Bit definition for CAN_RFIFO0 register  *******************/
#define CH9434D_CAN_RFIFO0_FMP0 ((u8_t)0x03)  /* FIFO 0 Message Pending */
#define CH9434D_CAN_RFIFO0_FULL0 ((u8_t)0x08) /* FIFO 0 Full */
#define CH9434D_CAN_RFIFO0_FOVR0 ((u8_t)0x10) /* FIFO 0 Overrun */
#define CH9434D_CAN_RFIFO0_RFOM0 ((u8_t)0x20) /* Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RFIFO1 register  *******************/
#define CH9434D_CAN_RFIFO1_FMP1 ((u8_t)0x03)  /* FIFO 1 Message Pending */
#define CH9434D_CAN_RFIFO1_FULL1 ((u8_t)0x08) /* FIFO 1 Full */
#define CH9434D_CAN_RFIFO1_FOVR1 ((u8_t)0x10) /* FIFO 1 Overrun */
#define CH9434D_CAN_RFIFO1_RFOM1 ((u8_t)0x20) /* Release FIFO 1 Output Mailbox */

// Definition of CAN register address
#define CH9434D_CAN_CTLR 0x00
/*******************  Bit definition for CAN_CTLR register  ********************/
#define CH9434D_CAN_CTLR_INRQ ((u32_t)0x00000001)  /* Initialization Request */
#define CH9434D_CAN_CTLR_SLEEP ((u32_t)0x00000002) /* Sleep Mode Request */
#define CH9434D_CAN_CTLR_TXFP ((u32_t)0x00000004)  /* Transmit FIFO Priority */
#define CH9434D_CAN_CTLR_RFLM ((u32_t)0x00000008)  /* Receive FIFO Locked Mode */
#define CH9434D_CAN_CTLR_NART ((u32_t)0x00000010)  /* No Automatic Retransmission */
#define CH9434D_CAN_CTLR_AWUM ((u32_t)0x00000020)  /* Automatic Wakeup Mode */
#define CH9434D_CAN_CTLR_ABOM ((u32_t)0x00000040)  /* Automatic Bus-Off Management */
#define CH9434D_CAN_CTLR_TTCM ((u32_t)0x00000080)  /* Time Triggered Communication Mode */
#define CH9434D_CAN_CTLR_RESET ((u32_t)0x00008000) /* CAN software master reset */
#define CH9434D_CAN_CTLR_DBF ((u32_t)0x00010000)   /* CAN controller operating state selection during debugging */

#define CH9434D_CAN_STATR 0x01
/*******************  Bit definition for CAN_STATR register  ********************/
#define CH9434D_CAN_STATR_INAK ((u16_t)0x0001)  /* Initialization Acknowledge */
#define CH9434D_CAN_STATR_SLAK ((u16_t)0x0002)  /* Sleep Acknowledge */
#define CH9434D_CAN_STATR_ERRI ((u16_t)0x0004)  /* Error Interrupt */
#define CH9434D_CAN_STATR_WKUI ((u16_t)0x0008)  /* Wakeup Interrupt */
#define CH9434D_CAN_STATR_SLAKI ((u16_t)0x0010) /* Sleep Acknowledge Interrupt */
#define CH9434D_CAN_STATR_TXM ((u16_t)0x0100)   /* Transmit Mode */
#define CH9434D_CAN_STATR_RXM ((u16_t)0x0200)   /* Receive Mode */
#define CH9434D_CAN_STATR_SAMP ((u16_t)0x0400)  /* Last Sample Point */
#define CH9434D_CAN_STATR_RX ((u16_t)0x0800)    /* CAN Rx Signal */

#define CH9434D_CAN_TSTATR 0x02
/*******************  Bit definition for CAN_TSTATR register  ********************/
#define CH9434D_CAN_TSTATR_RQCP0 ((u32_t)0x00000001) /* Request Completed Mailbox0 */
#define CH9434D_CAN_TSTATR_TXOK0 ((u32_t)0x00000002) /* Transmission OK of Mailbox0 */
#define CH9434D_CAN_TSTATR_ALST0 ((u32_t)0x00000004) /* Arbitration Lost for Mailbox0 */
#define CH9434D_CAN_TSTATR_TERR0 ((u32_t)0x00000008) /* Transmission Error of Mailbox0 */
#define CH9434D_CAN_TSTATR_ABRQ0 ((u32_t)0x00000080) /* Abort Request for Mailbox0 */

#define CH9434D_CAN_TSTATR_RQCP1 ((u32_t)0x00000100) /* Request Completed Mailbox1 */
#define CH9434D_CAN_TSTATR_TXOK1 ((u32_t)0x00000200) /* Transmission OK of Mailbox1 */
#define CH9434D_CAN_TSTATR_ALST1 ((u32_t)0x00000400) /* Arbitration Lost for Mailbox1 */
#define CH9434D_CAN_TSTATR_TERR1 ((u32_t)0x00000800) /* Transmission Error of Mailbox1 */
#define CH9434D_CAN_TSTATR_ABRQ1 ((u32_t)0x00008000) /* Abort Request for Mailbox 1 */

#define CH9434D_CAN_TSTATR_RQCP2 ((u32_t)0x00010000) /* Request Completed Mailbox2 */
#define CH9434D_CAN_TSTATR_TXOK2 ((u32_t)0x00020000) /* Transmission OK of Mailbox 2 */
#define CH9434D_CAN_TSTATR_ALST2 ((u32_t)0x00040000) /* Arbitration Lost for mailbox 2 */
#define CH9434D_CAN_TSTATR_TERR2 ((u32_t)0x00080000) /* Transmission Error of Mailbox 2 */
#define CH9434D_CAN_TSTATR_ABRQ2 ((u32_t)0x00800000) /* Abort Request for Mailbox 2 */

#define CH9434D_CAN_TSTATR_CODE ((u32_t)0x03000000)  /* Mailbox Code */

#define CH9434D_CAN_TSTATR_TME ((u32_t)0x1C000000)   /* TME[2:0] bits */
#define CH9434D_CAN_TSTATR_TME0 ((u32_t)0x04000000)  /* Transmit Mailbox 0 Empty */
#define CH9434D_CAN_TSTATR_TME1 ((u32_t)0x08000000)  /* Transmit Mailbox 1 Empty */
#define CH9434D_CAN_TSTATR_TME2 ((u32_t)0x10000000)  /* Transmit Mailbox 2 Empty */

#define CH9434D_CAN_TSTATR_LOW ((u32_t)0xE0000000)   /* LOW[2:0] bits */
#define CH9434D_CAN_TSTATR_LOW0 ((u32_t)0x20000000)  /* Lowest Priority Flag for Mailbox 0 */
#define CH9434D_CAN_TSTATR_LOW1 ((u32_t)0x40000000)  /* Lowest Priority Flag for Mailbox 1 */
#define CH9434D_CAN_TSTATR_LOW2 ((u32_t)0x80000000)  /* Lowest Priority Flag for Mailbox 2 */

#define CH9434D_CAN_RFIFO0 0x03
#define CH9434D_CAN_RFIFO1 0x04
/*******************  Bit definition for CAN_RFIFOx register  *******************/
#define CH9434D_CAN_RFIFOx_FMPx ((u32_t)0xFF)       /* FIFO x Message Pending */
#define CH9434D_CAN_RFIFOx_FULLx ((u32_t)(1 << 16)) /* FIFO x Full */
#define CH9434D_CAN_RFIFOx_FOVRx ((u32_t)(1 << 17)) /* FIFO x Overrun */
#define CH9434D_CAN_RFIFOx_RFOMx ((u32_t)(1 << 18)) /* Release FIFO x Output Mailbox */

#define CH9434D_CAN_INTENR 0x05
/********************  Bit definition for CAN_INTENR register  *******************/
#define CH9434D_CAN_INTENR_TMEIE ((u32_t)0x00000001)  /* Transmit Mailbox Empty Interrupt Enable */
#define CH9434D_CAN_INTENR_FMPIE0 ((u32_t)0x00000002) /* FIFO Message Pending Interrupt Enable */
#define CH9434D_CAN_INTENR_FFIE0 ((u32_t)0x00000004)  /* FIFO Full Interrupt Enable */
#define CH9434D_CAN_INTENR_FOVIE0 ((u32_t)0x00000008) /* FIFO Overrun Interrupt Enable */
#define CH9434D_CAN_INTENR_FMPIE1 ((u32_t)0x00000010) /* FIFO Message Pending Interrupt Enable */
#define CH9434D_CAN_INTENR_FFIE1 ((u32_t)0x00000020)  /* FIFO Full Interrupt Enable */
#define CH9434D_CAN_INTENR_FOVIE1 ((u32_t)0x00000040) /* FIFO Overrun Interrupt Enable */
#define CH9434D_CAN_INTENR_EWGIE ((u32_t)0x00000100)  /* Error Warning Interrupt Enable */
#define CH9434D_CAN_INTENR_EPVIE ((u32_t)0x00000200)  /* Error Passive Interrupt Enable */
#define CH9434D_CAN_INTENR_BOFIE ((u32_t)0x00000400)  /* Bus-Off Interrupt Enable */
#define CH9434D_CAN_INTENR_LECIE ((u32_t)0x00000800)  /* Last Error Code Interrupt Enable */
#define CH9434D_CAN_INTENR_ERRIE ((u32_t)0x00008000)  /* Error Interrupt Enable */
#define CH9434D_CAN_INTENR_WKUIE ((u32_t)0x00010000)  /* Wakeup Interrupt Enable */
#define CH9434D_CAN_INTENR_SLKIE ((u32_t)0x00020000)  /* Sleep Interrupt Enable */

#define CH9434D_CAN_ERRSR 0x06
/********************  Bit definition for CAN_ERRSR register  *******************/
#define CH9434D_CAN_ERRSR_EWGF ((u32_t)0x00000001)  /* Error Warning Flag */
#define CH9434D_CAN_ERRSR_EPVF ((u32_t)0x00000002)  /* Error Passive Flag */
#define CH9434D_CAN_ERRSR_BOFF ((u32_t)0x00000004)  /* Bus-Off Flag */

#define CH9434D_CAN_ERRSR_LEC ((u32_t)0x00000070)   /* LEC[2:0] bits (Last Error Code) */
#define CH9434D_CAN_ERRSR_LEC_0 ((u32_t)0x00000010) /* Bit 0 */
#define CH9434D_CAN_ERRSR_LEC_1 ((u32_t)0x00000020) /* Bit 1 */
#define CH9434D_CAN_ERRSR_LEC_2 ((u32_t)0x00000040) /* Bit 2 */

#define CH9434D_CAN_ERRSR_TEC ((u32_t)0x00FF0000)   /* Least significant byte of the 9-bit Transmit Error Counter */
#define CH9434D_CAN_ERRSR_REC ((u32_t)0xFF000000)   /* Receive Error Counter */

#define CH9434D_CAN_BTIMR 0x07
/*******************  Bit definition for CAN_BTIMR register  ********************/
#define CH9434D_CAN_BTIMR_BRP ((u32_t)0x000003FF)  /* Baud Rate Prescaler */
#define CH9434D_CAN_BTIMR_TS1 ((u32_t)0x000F0000)  /* Time Segment 1 */
#define CH9434D_CAN_BTIMR_TS2 ((u32_t)0x00700000)  /* Time Segment 2 */
#define CH9434D_CAN_BTIMR_SJW ((u32_t)0x03000000)  /* Resynchronization Jump Width */
#define CH9434D_CAN_BTIMR_LBKM ((u32_t)0x40000000) /* Loop Back Mode (Debug) */
#define CH9434D_CAN_BTIMR_SILM ((u32_t)0x80000000) /* Silent Mode */

#define CH9434D_CAN_TTCTLR 0x08
/********************  Bit definition for CAN_TTCTLR register  *******************/
#define CH9434D_CAN_TTCTLR_TIMCMV ((u32_t)0x0000FFFF)
#define CH9434D_CAN_TTCTLR_TIMRST ((u32_t)0x00010000)
#define CH9434D_CAN_TTCTLR_MODE ((u32_t)0x00020000)

#define CH9434D_CAN_TTCNT 0x09
/********************  Bit definition for CAN_TTCNT register  *******************/
// #define CH9434D_CAN_TTCNT ((u32_t)0x0000FFFF)

#define CH9434D_CAN_TERR_CNT 0x0A
/********************  Bit definition for CAN_TERR_CNT register  *******************/
// #define CH9434D_CAN_TERR_CNT ((u32_t)0x000001FF)

#define CH9434D_CAN_TXMIR0 0x0B
/******************  Bit definition for CAN_TXMI0R register  ********************/
#define CH9434D_CAN_TXMIRx_TXRQ ((u32_t)0x00000001) /* Transmit Mailbox Request */
#define CH9434D_CAN_TXMIRx_RTR ((u32_t)0x00000002)  /* Remote Transmission Request */
#define CH9434D_CAN_TXMIRx_IDE ((u32_t)0x00000004)  /* Identifier Extension */
#define CH9434D_CAN_TXMIRx_EXID ((u32_t)0x001FFFF8) /* Extended Identifier */
#define CH9434D_CAN_TXMIRx_STID ((u32_t)0xFFE00000) /* Standard Identifier or Extended Identifier */

#define CH9434D_CAN_TXMDTR0 0x0C
/******************  Bit definition for CAN_TXMDT0R register  *******************/
#define CH9434D_CAN_TXMDTRx_DLC ((u32_t)0x0000000F)  /* Data Length Code */
#define CH9434D_CAN_TXMDTRx_TGT ((u32_t)0x00000100)  /* Transmit Global Time */
#define CH9434D_CAN_TXMDTRx_TIME ((u32_t)0xFFFF0000) /* Message Time Stamp */

#define CH9434D_CAN_TXMDLR0 0x0D
/******************  Bit definition for CAN_TXMDL0R register  *******************/
#define CH9434D_CAN_TXMDLRx_DATA0 ((u32_t)0x000000FF) /* Data byte 0 */
#define CH9434D_CAN_TXMDLRx_DATA1 ((u32_t)0x0000FF00) /* Data byte 1 */
#define CH9434D_CAN_TXMDLRx_DATA2 ((u32_t)0x00FF0000) /* Data byte 2 */
#define CH9434D_CAN_TXMDLRx_DATA3 ((u32_t)0xFF000000) /* Data byte 3 */

#define CH9434D_CAN_TXMDHR0 0x0E
/******************  Bit definition for CAN_TXMDH0R register  *******************/
#define CH9434D_CAN_TXMDHRx_DATA4 ((u32_t)0x000000FF) /* Data byte 4 */
#define CH9434D_CAN_TXMDHRx_DATA5 ((u32_t)0x0000FF00) /* Data byte 5 */
#define CH9434D_CAN_TXMDHRx_DATA6 ((u32_t)0x00FF0000) /* Data byte 6 */
#define CH9434D_CAN_TXMDHRx_DATA7 ((u32_t)0xFF000000) /* Data byte 7 */


#define CH9434D_CAN_RX0READ 0x40
#define CH9434D_CAN_RX1READ 0x41
#define CH9434D_CAN_TX0WRITE 0x42
#define CH9434D_CAN_TX1WRITE 0x43
#define CH9434D_CAN_TX2WRITE 0x44


#define CH9434D_CAN_TXMIR1 0x0F
#define CH9434D_CAN_TXMDTR1 0x10
#define CH9434D_CAN_TXMDLR1 0x11
#define CH9434D_CAN_TXMDHR1 0x12

#define CH9434D_CAN_TXMIR2 0x13
#define CH9434D_CAN_TXMDTR2 0x14
#define CH9434D_CAN_TXMDLR2 0x15
#define CH9434D_CAN_TXMDHR2 0x16

#define CH9434D_CAN_RXMIR0 0x17
/*******************  Bit definition for CAN_RXMI0R register  *******************/
// #define CH9434D_CAN_RXMIOR_FDF                          ((u32_t)0x00000001)
#define CH9434D_CAN_RXMIRx_RTR ((u32_t)0x00000002)  /* Remote Transmission Request */
#define CH9434D_CAN_RXMIRx_IDE ((u32_t)0x00000004)  /* Identifier Extension */
#define CH9434D_CAN_RXMIRx_EXID ((u32_t)0x001FFFF8) /* Extended Identifier */
#define CH9434D_CAN_RXMIRx_STID ((u32_t)0xFFE00000) /* Standard Identifier or Extended Identifier */

#define CH9434D_CAN_RXMDTR0 0x18
/*******************  Bit definition for CAN_RXMDT0R register  ******************/
#define CH9434D_CAN_RXMDTRx_DLC ((u32_t)0x0000000F) /* Data Length Code */
#define CH9434D_CAN_RXMDTRx_FMI ((u32_t)0x0000FF00)  /* Filter Match Index */
#define CH9434D_CAN_RXMDTRx_TIME ((u32_t)0xFFFF0000) /* Message Time Stamp */

#define CH9434D_CAN_RXMDLR0 0x19
/*******************  Bit definition for CAN_RXMDL0R register  ******************/
#define CH9434D_CAN_RXMDLRx_DATA0 ((u32_t)0x000000FF) /* Data byte 0 */
#define CH9434D_CAN_RXMDLRx_DATA1 ((u32_t)0x0000FF00) /* Data byte 1 */
#define CH9434D_CAN_RXMDLRx_DATA2 ((u32_t)0x00FF0000) /* Data byte 2 */
#define CH9434D_CAN_RXMDLRx_DATA3 ((u32_t)0xFF000000) /* Data byte 3 */

#define CH9434D_CAN_RXMDHR0 0x1A
/*******************  Bit definition for CAN_RXMDH0R register  ******************/
#define CH9434D_CAN_RXMDHRx_DATA4 ((u32_t)0x000000FF) /* Data byte 4 */
#define CH9434D_CAN_RXMDHRx_DATA5 ((u32_t)0x0000FF00) /* Data byte 5 */
#define CH9434D_CAN_RXMDHRx_DATA6 ((u32_t)0x00FF0000) /* Data byte 6 */
#define CH9434D_CAN_RXMDHRx_DATA7 ((u32_t)0xFF000000) /* Data byte 7 */

#define CH9434D_CAN_RXMIR1 0x1B
#define CH9434D_CAN_RXMDTR1 0x1C
#define CH9434D_CAN_RXMDLR1 0x1D
#define CH9434D_CAN_RXMDHR1 0x1E

#define CH9434D_CAN_FCTLR 0x1F
/*******************  Bit definition for CAN_FCTLR register  ********************/
#define CH9434D_CAN_FCTLR_FINIT ((u8_t)0x01) /* Filter Init Mode */

#define CH9434D_CAN_FMCFGR 0x20
/*******************  Bit definition for CAN_FMCFGR register  *******************/
#define CH9434D_CAN_FMCFGR_FBM_MASK ((u32_t)0x00003FFF) /* Filter Mode */
#define CH9434D_CAN_FMCFGR_FBM(x) ((u32_t)(1 << x))     /* Filter Init Mode bit x */

#define CH9434D_CAN_FSCFGR 0x21
/*******************  Bit definition for CAN_FSCFGR register  *******************/
#define CH9434D_CAN_FSCFGR_FSC_MASK ((u32_t)0x00003FFF) /* Filter Scale Configuration */
#define CH9434D_CAN_FSCFGR_FSC(x) ((u32_t)(1 << x))     /* Filter Scale Configuration bit x */

#define CH9434D_CAN_FAFIFOR 0x22
/******************  Bit definition for CAN_FAFIFOR register  *******************/
#define CH9434D_CAN_FAFIFOR_FFA_MASK ((u32_t)0x00003FFF) /* Filter FIFO Assignment */
#define CH9434D_CAN_FAFIFOR_FFA(x) ((u32_t)(1 << x))     /* Filter FIFO Assignment for Filter x */

#define CH9434D_CAN_FWR 0x23
/*******************  Bit definition for CAN_FWR register  *******************/
#define CH9434D_CAN_FWR_FACT_MASK ((u32_t)0x00003FFF) /* Filter Active */
#define CH9434D_CAN_FWR_FACT(x) ((u32_t)(1 << x))     /* Filter x Active */

#define CH9434D_CAN_F0R1 0x24
#define CH9434D_CAN_F0R2 0x25
#define CH9434D_CAN_F1R1 0x26
#define CH9434D_CAN_F1R2 0x27
#define CH9434D_CAN_F2R1 0x28
#define CH9434D_CAN_F2R2 0x29
#define CH9434D_CAN_F3R1 0x2A
#define CH9434D_CAN_F3R2 0x2B
#define CH9434D_CAN_F4R1 0x2C
#define CH9434D_CAN_F4R2 0x2D
#define CH9434D_CAN_F5R1 0x2E
#define CH9434D_CAN_F5R2 0x2F
#define CH9434D_CAN_F6R1 0x30
#define CH9434D_CAN_F6R2 0x31
#define CH9434D_CAN_F7R1 0x32
#define CH9434D_CAN_F7R2 0x33
#define CH9434D_CAN_F8R1 0x34
#define CH9434D_CAN_F8R2 0x35
#define CH9434D_CAN_F9R1 0x36
#define CH9434D_CAN_F9R2 0x37
#define CH9434D_CAN_F10R1 0x38
#define CH9434D_CAN_F10R2 0x39
#define CH9434D_CAN_F11R1 0x3A
#define CH9434D_CAN_F11R2 0x3B
#define CH9434D_CAN_F12R1 0x3C
#define CH9434D_CAN_F12R2 0x3D
#define CH9434D_CAN_F13R1 0x3E
#define CH9434D_CAN_F13R2 0x3F
/*******************  Bit definition for CAN_FnRm register  *******************/
#define CH9434D_CAN_FnRm_FB(x) ((u32_t)(1 << x)) /* Filter bit x */
#define CH9434D_CAN_FxR1(x) (0x24 + x * 2)
#define CH9434D_CAN_FxR2(x) (0x25 + x * 2)

// CAN register continuous operation command
#define CH9434D_CAN_RX0READ 0x40
#define CH9434D_CAN_RX1READ 0x41
#define CH9434D_CAN_TX0WRITE 0x42
#define CH9434D_CAN_TX1WRITE 0x43
#define CH9434D_CAN_TX2WRITE 0x44

/* -----------------------------------------------------------------------------
 *                         CAN-related type definitions
 * -----------------------------------------------------------------------------
 */

typedef enum { DISABLE_T = 0,
               ENABLE_T = !DISABLE_T } CH9434_FuncSta;

typedef enum { RESET_T = 0,
               SET_T = !RESET_T } CH9434_FSta,
    CH9434_ISta;

/* CAN init structure definition */
typedef struct
{
    u16_t CAN_Prescaler;     /* Specifies the length of a time quantum.
                                   It ranges from 1 to 1024. */

    u8_t CAN_Mode;           /* Specifies the CAN operating mode.
                                   This parameter can be a value of
                                  @ref CAN_operating_mode */

    u8_t CAN_SJW;            /* Specifies the maximum number of time quanta
                                   the CAN hardware is allowed to lengthen or
                                   shorten a bit to perform resynchronization.
                                   This parameter can be a value of
                                   @ref CAN_synchronisation_jump_width */

    u8_t CAN_BS1;            /* Specifies the number of time quanta in Bit
                                   Segment 1. This parameter can be a value of
                                   @ref CAN_time_quantum_in_bit_segment_1 */

    u8_t CAN_BS2;            /* Specifies the number of time quanta in Bit
                                   Segment 2.
                                   This parameter can be a value of
                                   @ref CAN_time_quantum_in_bit_segment_2 */

    CH9434_FuncSta CAN_TTCM; /* Enable or disable the time triggered
                                 communication mode. This parameter can be set
                                 either to ENABLE or DISABLE. */

    CH9434_FuncSta CAN_ABOM; /* Enable or disable the automatic bus-off
                                 management. This parameter can be set either
                                 to ENABLE or DISABLE. */

    CH9434_FuncSta CAN_AWUM; /* Enable or disable the automatic wake-up mode.
                                 This parameter can be set either to ENABLE or
                                 DISABLE. */

    CH9434_FuncSta CAN_NART; /* Enable or disable the no-automatic
                                 retransmission mode. This parameter can be
                                 set either to ENABLE or DISABLE. */

    CH9434_FuncSta CAN_RFLM; /* Enable or disable the Receive FIFO Locked mode.
                                 This parameter can be set either to ENABLE
                                 or DISABLE. */

    CH9434_FuncSta CAN_TXFP; /* Enable or disable the transmit FIFO priority.
                                 This parameter can be set either to ENABLE
                                 or DISABLE. */
} CH9434D_CAN_InitTypeDef;

/* CAN filter init structure definition */
typedef struct
{
    u16_t CAN_FilterIdHigh;              /* Specifies the filter identification number (MSBs for a 32-bit
                                                   configuration, first one for a 16-bit configuration).
                                                   This parameter can be a value between 0x0000 and 0xFFFF */

    u16_t CAN_FilterIdLow;               /* Specifies the filter identification number (LSBs for a 32-bit
                                                   configuration, second one for a 16-bit configuration).
                                                   This parameter can be a value between 0x0000 and 0xFFFF */

    u16_t CAN_FilterMaskIdHigh;          /* Specifies the filter mask number or identification number,
                                                   according to the mode (MSBs for a 32-bit configuration,
                                                   first one for a 16-bit configuration).
                                                   This parameter can be a value between 0x0000 and 0xFFFF */

    u16_t CAN_FilterMaskIdLow;           /* Specifies the filter mask number or identification number,
                                                   according to the mode (LSBs for a 32-bit configuration,
                                                   second one for a 16-bit configuration).
                                                   This parameter can be a value between 0x0000 and 0xFFFF */

    u16_t CAN_FilterFIFOAssignment;      /* Specifies the FIFO (0 or 1) which will be assigned to the filter.
                                                   This parameter can be a value of @ref CAN_filter_FIFO */

    u8_t CAN_FilterNumber;               /* Specifies the filter which will be initialized. It ranges from 0 to 13. */

    u8_t CAN_FilterMode;                 /* Specifies the filter mode to be initialized.
                                                   This parameter can be a value of @ref CAN_filter_mode */

    u8_t CAN_FilterScale;                /* Specifies the filter scale.
                                                   This parameter can be a value of @ref CAN_filter_scale */

    CH9434_FuncSta CAN_FilterActivation; /* Enable or disable the filter.
                                              This parameter can be set either to ENABLE or DISABLE. */
} CH9434D_CAN_FilterInitTypeDef;

/* CAN Tx message structure definition */
typedef struct
{
    u32_t StdId;  /* Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

    u32_t ExtId;  /* Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

    u8_t IDE;     /* Specifies the type of identifier for the message that
                        will be transmitted. This parameter can be a value
                        of @ref CAN_identifier_type */

    u8_t RTR;     /* Specifies the type of frame for the message that will
                        be transmitted. This parameter can be a value of
                        @ref CAN_remote_transmission_request */

    u8_t DLC;     /* Specifies the length of the frame that will be
                        transmitted. This parameter can be a value between
                        0 to 8 */

    u8_t Data[8]; /* Contains the data to be transmitted. It ranges from 0
                         to 0xFF. */
} CH9434D_CanTxMsg;

/* CAN Rx message structure definition  */
typedef struct
{
    u32_t StdId;  /* Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

    u32_t ExtId;  /* Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

    u8_t IDE;     /* Specifies the type of identifier for the message that
                        will be received. This parameter can be a value of
                        @ref CAN_identifier_type */

    u8_t RTR;     /* Specifies the type of frame for the received message.
                        This parameter can be a value of
                        @ref CAN_remote_transmission_request */

    u8_t DLC;     /* Specifies the length of the frame that will be received.
                        This parameter can be a value between 0 to 8 */

    u8_t Data[8]; /* Contains the data to be received. It ranges from 0 to
                        0xFF. */

    u8_t FMI;     /* Specifies the index of the filter the message stored in
                        the mailbox passes through. This parameter can be a
                        value between 0 to 0xFF */
} CH9434D_CanRxMsg;

/* CAN_sleep_constants */
#define CH9434D_CAN_InitStatus_Failed ((u8_t)0x00)  /* CAN initialization failed */
#define CH9434D_CAN_InitStatus_Success ((u8_t)0x01) /* CAN initialization OK */

/* CAN_Mode */
#define CH9434D_CAN_Mode_Normal ((u8_t)0x00)          /* normal mode */
#define CH9434D_CAN_Mode_LoopBack ((u8_t)0x01)        /* loopback mode */
#define CH9434D_CAN_Mode_Silent ((u8_t)0x02)          /* silent mode */
#define CH9434D_CAN_Mode_Silent_LoopBack ((u8_t)0x03) /* loopback combined with silent mode */

/* CAN_Operating_Mode */
#define CH9434D_CAN_OperatingMode_Initialization ((u8_t)0x00) /* Initialization mode */
#define CH9434D_CAN_OperatingMode_Normal ((u8_t)0x01)         /* Normal mode */
#define CH9434D_CAN_OperatingMode_Sleep ((u8_t)0x02)          /* sleep mode */

/* CAN_Mode_Status */
#define CH9434D_CAN_ModeStatus_Failed ((u8_t)0x00)                            /* CAN entering the specific mode failed */
#define CH9434D_CAN_ModeStatus_Success ((u8_t)!CH9434D_CAN_ModeStatus_Failed) /* CAN entering the specific mode Succeed */

/* CAN_synchronisation_jump_width */
#define CH9434D_CAN_SJW_tq(x) ((u8_t)(x - 1) % 4) /* x time quantum x:(1-4) */

/* CAN_time_quantum_in_bit_segment_1 */
#define CH9434D_CAN_BS1_tq(x) ((u8_t)(x - 1) % 16) /* x time quantum x:(1-16) */

/* CAN_time_quantum_in_bit_segment_2 */
#define CH9434D_CAN_BS2_tq(x) ((u8_t)(x - 1) % 8) /* x time quantum x:(1-16) */

/* CAN_filter_mode */
#define CH9434D_CAN_FilterMode_IdMask ((u8_t)0x00) /* identifier/mask mode */
#define CH9434D_CAN_FilterMode_IdList ((u8_t)0x01) /* identifier list mode */

/* CAN_filter_scale */
#define CH9434D_CAN_FilterScale_16bit ((u8_t)0x00) /* Two 16-bit filters */
#define CH9434D_CAN_FilterScale_32bit ((u8_t)0x01) /* One 32-bit filter */

/* CAN_filter_FIFO */
#define CH9434D_CAN_Filter_FIFO0 ((u8_t)0x00) /* Filter FIFO 0 assignment for filter x */
#define CH9434D_CAN_Filter_FIFO1 ((u8_t)0x01) /* Filter FIFO 1 assignment for filter x */

/* CAN_identifier_type */
#define CH9434D_CAN_Id_Standard ((u32_t)0x00000000) /* Standard Id */
#define CH9434D_CAN_Id_Extended ((u32_t)0x00000004) /* Extended Id */

/* CAN_remote_transmission_request */
#define CH9434D_CAN_RTR_Data ((u32_t)0x00000000)   /* Data frame */
#define CH9434D_CAN_RTR_Remote ((u32_t)0x00000002) /* Remote frame */

/* CAN_transmit_constants */
#define CH9434D_CAN_TxStatus_Failed ((u8_t)0x00)    /* CAN transmission failed */
#define CH9434D_CAN_TxStatus_Ok ((u8_t)0x01)        /* CAN transmission succeeded */
#define CH9434D_CAN_TxStatus_Pending ((u8_t)0x02)   /* CAN transmission pending */
#define CH9434D_CAN_TxStatus_NoMailBox ((u8_t)0x04) /* CAN cell did not provide an empty mailbox */

/* CAN_receive_FIFO_number_constants */
#define CH9434D_CAN_FIFO0 ((u8_t)0x00) /* CAN FIFO 0 used to receive */
#define CH9434D_CAN_FIFO1 ((u8_t)0x01) /* CAN FIFO 1 used to receive */

/* CAN_sleep_constants */
#define CH9434D_CAN_Sleep_Failed ((u8_t)0x00) /* CAN did not enter the sleep mode */
#define CH9434D_CAN_Sleep_Ok ((u8_t)0x01)     /* CAN entered the sleep mode */

/* CAN_wake_up_constants */
#define CH9434D_CAN_WakeUp_Failed ((u8_t)0x00) /* CAN did not leave the sleep mode */
#define CH9434D_CAN_WakeUp_Ok ((u8_t)0x01)     /* CAN leaved the sleep mode */

/* CAN_Error_Code_constants */
#define CH9434D_CAN_ErrorCode_NoErr ((u8_t)0x00)           /* No Error */
#define CH9434D_CAN_ErrorCode_StuffErr ((u8_t)0x10)        /* Stuff Error */
#define CH9434D_CAN_ErrorCode_FormErr ((u8_t)0x20)         /* Form Error */
#define CH9434D_CAN_ErrorCode_ACKErr ((u8_t)0x30)          /* Acknowledgment Error */
#define CH9434D_CAN_ErrorCode_BitRecessiveErr ((u8_t)0x40) /* Bit Recessive Error */
#define CH9434D_CAN_ErrorCode_BitDominantErr ((u8_t)0x50)  /* Bit Dominant Error */
#define CH9434D_CAN_ErrorCode_CRCErr ((u8_t)0x60)          /* CRC Error  */
#define CH9434D_CAN_ErrorCode_SoftwareSetErr ((u8_t)0x70)  /* Software Set Error */

/* CAN_flags */
/* Transmit Flags */
/* If the flag is 0x3XXXXXXX, it means that it can be used with CAN_GetFlagStatus()
 * and CAN_ClearFlag() functions.
 * If the flag is 0x1XXXXXXX, it means that it can only be used with CAN_GetFlagStatus() function.
 */
#define CH9434D_CAN_FLAG_RQCP0 ((u32_t)0x38000001) /* Request MailBox0 Flag */
#define CH9434D_CAN_FLAG_RQCP1 ((u32_t)0x38000100) /* Request MailBox1 Flag */
#define CH9434D_CAN_FLAG_RQCP2 ((u32_t)0x38010000) /* Request MailBox2 Flag */

/* Receive Flags */
#define CH9434D_CAN_FLAG_FMP0 ((u32_t)0x12000003) /* FIFO 0 Message Pending Flag */
#define CH9434D_CAN_FLAG_FF0 ((u32_t)0x32000008)  /* FIFO 0 Full Flag            */
#define CH9434D_CAN_FLAG_FOV0 ((u32_t)0x32000010) /* FIFO 0 Overrun Flag         */
#define CH9434D_CAN_FLAG_FMP1 ((u32_t)0x14000003) /* FIFO 1 Message Pending Flag */
#define CH9434D_CAN_FLAG_FF1 ((u32_t)0x34000008)  /* FIFO 1 Full Flag            */
#define CH9434D_CAN_FLAG_FOV1 ((u32_t)0x34000010) /* FIFO 1 Overrun Flag         */

/* Operating Mode Flags */
#define CH9434D_CAN_FLAG_WKU ((u32_t)0x31000008)  /* Wake up Flag */
#define CH9434D_CAN_FLAG_SLAK ((u32_t)0x31000012) /* Sleep acknowledge Flag */
/* Note:
 *When SLAK intterupt is disabled (SLKIE=0), no polling on SLAKI is possible.
 *In this case the SLAK bit can be polled.
 */

/* Error Flags */
#define CH9434D_CAN_FLAG_EWG ((u32_t)0x10F00001) /* Error Warning Flag   */
#define CH9434D_CAN_FLAG_EPV ((u32_t)0x10F00002) /* Error Passive Flag   */
#define CH9434D_CAN_FLAG_BOF ((u32_t)0x10F00004) /* Bus-Off Flag         */
#define CH9434D_CAN_FLAG_LEC ((u32_t)0x30F00070) /* Last error code Flag */

/* CAN_interrupts */
#define CH9434D_CAN_IT_TME ((u32_t)0x00000001) /* Transmit mailbox empty Interrupt*/

/* Receive Interrupts */
#define CH9434D_CAN_IT_FMP0 ((u32_t)0x00000002) /* FIFO 0 message pending Interrupt*/
#define CH9434D_CAN_IT_FF0 ((u32_t)0x00000004)  /* FIFO 0 full Interrupt*/
#define CH9434D_CAN_IT_FOV0 ((u32_t)0x00000008) /* FIFO 0 overrun Interrupt*/
#define CH9434D_CAN_IT_FMP1 ((u32_t)0x00000010) /* FIFO 1 message pending Interrupt*/
#define CH9434D_CAN_IT_FF1 ((u32_t)0x00000020)  /* FIFO 1 full Interrupt*/
#define CH9434D_CAN_IT_FOV1 ((u32_t)0x00000040) /* FIFO 1 overrun Interrupt*/

/* Operating Mode Interrupts */
#define CH9434D_CAN_IT_WKU ((u32_t)0x00010000) /* Wake-up Interrupt*/
#define CH9434D_CAN_IT_SLK ((u32_t)0x00020000) /* Sleep acknowledge Interrupt*/

/* Error Interrupts */
#define CH9434D_CAN_IT_EWG ((u32_t)0x00000100) /* Error warning Interrupt*/
#define CH9434D_CAN_IT_EPV ((u32_t)0x00000200) /* Error passive Interrupt*/
#define CH9434D_CAN_IT_BOF ((u32_t)0x00000400) /* Bus-off Interrupt*/
#define CH9434D_CAN_IT_LEC ((u32_t)0x00000800) /* Last error code Interrupt*/
#define CH9434D_CAN_IT_ERR ((u32_t)0x00008000) /* Error Interrupt*/

/* Flags named as Interrupts : kept only for FW compatibility */
#define CH9434D_CAN_IT_RQCP0 CH9434D_CAN_IT_TME
#define CH9434D_CAN_IT_RQCP1 CH9434D_CAN_IT_TME
#define CH9434D_CAN_IT_RQCP2 CH9434D_CAN_IT_TME

/* CAN_BS1_Mode */
#define CH9434D_CAN_BS1_4bit ((u32_t)0x00000000)
#define CH9434D_CAN_BS1_6bit ((u32_t)0x00000100)

/* CAN_Transmit_Mailbox_number_constants */
#define CH9434D_CAN_Transmit_Mailbox0 ((u8_t)0x00)
#define CH9434D_CAN_Transmit_Mailbox1 ((u8_t)0x01)
#define CH9434D_CAN_Transmit_Mailbox2 ((u8_t)0x02)

/* Time out for INAK bit */
#define CH9434D_INAK_TIMEOUT 10  //((u32_t)0x0000FFFF)
/* Time out for SLAK bit */
#define CH9434D_SLAK_TIMEOUT ((u32_t)0x0000FFFF)

/* Flags in TSTATR register */
#define CH9434D_CAN_FLAGS_TSTATR ((u32_t)0x08000000)
/* Flags in RFIFO1 register */
#define CH9434D_CAN_FLAGS_RFIFO1 ((u32_t)0x04000000)
/* Flags in RFIFO0 register */
#define CH9434D_CAN_FLAGS_RFIFO0 ((u32_t)0x02000000)
/* Flags in STATR register */
#define CH9434D_CAN_FLAGS_STATR ((u32_t)0x01000000)
/* Flags in ERRSR register */
#define CH9434D_CAN_FLAGS_ERRSR ((u32_t)0x00F00000)


/* -----------------------------------------------------------------------------
 *                             Interface function
 * -----------------------------------------------------------------------------
 */
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
void CH9434IICAddSet (u8_t set_iic_add);
#endif  //(USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
void CH9434OscXtFreqSet (u32_t x_freq);
void CH9434InitClkMode (u8_t xt_en, u8_t freq_mul_en, u8_t div_num);
void CH9434UARTxParaSet (u8_t uart_idx, u32_t bps, u8_t data_bits, u8_t stop_bits, u8_t veri_bits);
void CH9434UARTxFIFOSet (u8_t uart_idx, u8_t fifo_en, u8_t fifo_level);
void CH9434UARTxIrqSet (u8_t uart_idx, u8_t modem, u8_t line, u8_t tx, u8_t rx);
void CH9434UARTxFlowSet (u8_t uart_idx, u8_t flow_en);
void CH9434UARTxIrqOpen (u8_t uart_idx);
void CH9434UARTxRtsDtrPin (u8_t uart_idx, u8_t rts_val, u8_t dtr_val);
void CH9434UARTxWriteSRC (u8_t uart_idx, u8_t src_val);
u8_t CH9434UARTxReadSRC (u8_t uart_idx);
u8_t CH9434UARTxReadIIR (u8_t uart_idx);
u8_t CH9434UARTxReadLSR (u8_t uart_idx);
u8_t CH9434UARTxReadMSR (u8_t uart_idx);
u16_t CH9434UARTxGetRxFIFOLen (u8_t uart_idx);
u8_t CH9434UARTxGetRxFIFOData (u8_t uart_idx, u8_t *p_data, u16_t read_len);
u16_t CH9434UARTxGetTxFIFOLen (u8_t uart_idx);
u8_t CH9434UARTxSetTxFIFOData (u8_t uart_idx, u8_t *p_data, u16_t send_len);
void CH9434UARTxTnowSet (u8_t uart_idx, u8_t tnow_en, u8_t polar);
void CH9434LowerPowerModeSet (u8_t mode);
void CH9434WakeUp (void);
void CH9434GPIOFuncSet (u8_t gpio_idx, u8_t en, u8_t dir, u8_t pu, u8_t pd);
void CH9434GPIOPinOut (u8_t gpio_idx, u8_t out_val);
u8_t CH9434GPIOPinVal (u8_t gpio_idx);

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
void CH9434DefIOFuncEn (u8_t io_idx);
u8_t CH9434ReadDefIOFuncEnSta (u8_t io_idx);
void CH9434MulIOFuncEn (u8_t io_idx);
u8_t CH9434ReadMulIOFuncEnSta (u8_t io_idx);

u8_t CH9434D_CAN_Init (CH9434D_CAN_InitTypeDef *CAN_InitStruct);
void CH9434D_CAN_FilterInit (CH9434D_CAN_FilterInitTypeDef *CAN_FilterInitStruct);
void CH9434D_CAN_StructInit (CH9434D_CAN_InitTypeDef *CAN_InitStruct);
void CH9434D_CAN_TTComModeCmd (CH9434_FuncSta NewState);
u8_t CH9434D_CAN_Transmit (CH9434D_CanTxMsg *TxMessage);
u8_t CH9434D_CAN_TransmitStatus (u8_t TransmitMailbox);
void CH9434D_CAN_CancelTransmit (u8_t Mailbox);
void CH9434D_CAN_Receive (u8_t FIFONumber, CH9434D_CanRxMsg *RxMessage);
void CH9434D_CAN_FIFORelease (u8_t FIFONumber);
u8_t CH9434D_CAN_MessagePending (u8_t FIFONumber);
u8_t CH9434D_CAN_OperatingModeRequest (u8_t CAN_OperatingMode);
u8_t CH9434D_CAN_Sleep (void);
u8_t CH9434D_CAN_WakeUp (void);
u8_t CH9434D_CAN_GetLastErrorCode (void);
u8_t CH9434D_CAN_GetReceiveErrorCounter (void);
u8_t CH9434D_CAN_GetLSBTransmitErrorCounter (void);
void CH9434D_CAN_ITConfig (u32_t CAN_IT, CH9434_FuncSta NewState);
CH9434_FSta CH9434D_CAN_GetFlagStatus (u32_t CAN_FLAG);
void CH9434D_CAN_ClearFlag (u32_t CAN_FLAG);
CH9434_ISta CH9434D_CAN_GetITStatus (u32_t CAN_IT);
void CH9434D_CAN_ClearITPendingBit (u32_t CAN_IT);
CH9434_ISta CheckITStatus (u32_t CAN_Reg, u32_t It_Bit);
void CH9434D_CAN_BS1_ModeConfig (u32_t CAN_BS1_Mode, u8_t CH9434D_CAN_BS1_tq);
void CH9434D_CAN_BusOff_ErrCntConfig (u8_t BusOff_ErrCnt);
void CAN_TransmitDirect (u8_t mailbox, CH9434D_CanTxMsg *TxMessage);
void CAN_ReceiveDirect (u8_t FIFONumber, CH9434D_CanRxMsg *RxMessage);
#endif  //(USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif