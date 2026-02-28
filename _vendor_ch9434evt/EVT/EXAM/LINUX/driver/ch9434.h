
#include "linux/version.h"
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/gpio/driver.h>
#include <linux/netdevice.h>
#include <linux/uaccess.h>
#include <linux/can/core.h>
#include <linux/list.h>
#include <linux/can/dev.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0))
#include <linux/can/led.h>
#endif
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC "SPI/I2C to serial/CAN/GPIO driver for ch9434."
#define VERSION_DESC "V1.4 On 2025.08"

/******** CH9434D SPI or I2C control interface select *******/
#define USE_SPI_MODE
// #define USE_I2C_MODE

/******** CH9434D External clock or internal clock select *******/
#define EXTERN_CLOCK
// #define INTERNAL_CLOCK

/******** CH9434D TNOW function enable *******/
#define CH9434D_TNOW0_ON
#define CH9434D_TNOW1_ON
#define CH9434D_TNOW2_ON
#define CH9434D_TNOW3_ON
#undef CH9434D_TNOW0_ON
#undef CH9434D_TNOW1_ON
#undef CH9434D_TNOW2_ON
#undef CH9434D_TNOW3_ON

/******** CH9434A/M TNOW function enable *******/
#define CH943X_TNOW0_ON
#define CH943X_TNOW1_ON
#define CH943X_TNOW2_ON
#define CH943X_TNOW3_ON
#undef CH943X_TNOW0_ON
#undef CH943X_TNOW1_ON
#undef CH943X_TNOW2_ON
#undef CH943X_TNOW3_ON

/******** CH9434D CAN function switch *******/
#define CH9434D_CAN_ON
// #undef CH9434D_CAN_ON
/******** CH9434D CAN Continuous transmission mode *******/
#define CAN_RX_CONTMODE
#define CAN_TX_CONTMODE
#undef CAN_RX_CONTMODE
#undef CAN_TX_CONTMODE

/******** Interrupt IO config *******/
#define USE_IRQ_FROM_DTS
#define GPIO_NUMBER 0

#define CH943X_NAME_SPI "ch943x_spi"
#define CH943X_NAME_I2C "ch943x_i2c"

#define IOCTL_MAGIC 'W'
#define IOCTL_CMD_GPIOENABLE _IOW(IOCTL_MAGIC, 0x80, uint16_t)
#define IOCTL_CMD_GPIODIR _IOW(IOCTL_MAGIC, 0x81, uint16_t)
#define IOCTL_CMD_GPIOPULLUP _IOW(IOCTL_MAGIC, 0x82, uint16_t)
#define IOCTL_CMD_GPIOPULLDOWN _IOW(IOCTL_MAGIC, 0x83, uint16_t)
#define IOCTL_CMD_GPIOSET _IOW(IOCTL_MAGIC, 0x84, uint16_t)
#define IOCTL_CMD_GPIOGET _IOWR(IOCTL_MAGIC, 0x85, uint16_t)

#define IOCTL_CMD_CH9434D_GPIOENABLE _IOWR(IOCTL_MAGIC, 0x86, uint16_t)

#define IOCTL_CTRL_WRITE _IOWR(IOCTL_MAGIC, 0x87, uint16_t)
#define IOCTL_CTRL_READ _IOWR(IOCTL_MAGIC, 0x88, uint16_t)
#define IOCTL_CMD_GETCHIPTYPE _IOWR(IOCTL_MAGIC, 0x89, uint16_t)

#define VER_LEN 4

#define CH943X_REG_OP_WRITE 0x80
#define CH943X_REG_OP_READ 0x00

/* CH943X register definitions */
#define CH943X_RHR_REG (0x00) /* RX FIFO */
#define CH943X_THR_REG (0x00) /* TX FIFO */
#define CH943X_IER_REG (0x01) /* Interrupt enable */
#define CH943X_IIR_REG (0x02) /* Interrupt Identification */
#define CH943X_FCR_REG (0x02) /* FIFO control */
#define CH943X_LCR_REG (0x03) /* Line Control */
#define CH943X_MCR_REG (0x04) /* Modem Control */
#define CH943X_LSR_REG (0x05) /* Line Status */
#define CH943X_MSR_REG (0x06) /* Modem Status */
#define CH943X_SPR_REG (0x07) /* Scratch Pad */

#define CH943X_CLK_REG (0x48) /* Clock Set */
#define CH943X_RS485_REG (0x41) /* RS485 Control */
#define CH943X_FIFO_REG (0x42) /* FIFO Control */
#define CH943X_FIFOCL_REG (0x43) /* FIFO Count Low */
#define CH943X_FIFOCH_REG (0x44) /* FIFO Count High */

#define CH943X_GPIOEN_REG (0x50) /* GPIO Enable Set */
#define CH943X_GPIODIR_REG (0x54) /* GPIO Direction Set */
#define CH943X_GPIOPU_REG (0x58) /* GPIO PullUp Set */
#define CH943X_GPIOPD_REG (0x5C) /* GPIO PullDown Set */
#define CH943X_GPIOVAL_REG (0x60) /* GPIO Value Set */

#define CH943X_SPI_CONT_MODE_REG (0x64) /* SPI transfer mode Set */
#define CH943X_CHIP_VER_REG (0x65) /* Firmware Version */

/* Special Register set: Only if (LCR[7] == 1) */
#define CH943X_DLL_REG (0x00) /* Divisor Latch Low */
#define CH943X_DLH_REG (0x01) /* Divisor Latch High */

/* IER register bits */
#define CH943X_IER_RDI_BIT (1 << 0) /* Enable RX data interrupt */
#define CH943X_IER_THRI_BIT \
	(1 << 1) /* Enable TX holding register interrupt */
#define CH943X_IER_RLSI_BIT (1 << 2) /* Enable RX line status interrupt */
#define CH943X_IER_MSI_BIT (1 << 3) /* Enable Modem status interrupt */

/* IER enhanced register bits */
#define CH943X_IER_RESET_BIT (1 << 7) /* Enable Soft reset */
#define CH943X_IER_LOWPOWER_BIT (1 << 6) /* Enable low power mode */
#define CH943X_IER_SLEEP_BIT (1 << 5) /* Enable sleep mode */

/* FCR register bits */
#define CH943X_FCR_FIFO_BIT (1 << 0) /* Enable FIFO */
#define CH943X_FCR_RXRESET_BIT (1 << 1) /* Reset RX FIFO */
#define CH943X_FCR_TXRESET_BIT (1 << 2) /* Reset TX FIFO */
#define CH943X_FCR_RXLVLL_BIT (1 << 6) /* RX Trigger level LSB */
#define CH943X_FCR_RXLVLH_BIT (1 << 7) /* RX Trigger level MSB */

/* IIR register bits */
#define CH943X_IIR_NO_INT_BIT (1 << 0) /* No interrupts pending */
#define CH943X_IIR_ID_MASK 0x0e /* Mask for the interrupt ID */
#define CH943X_IIR_THRI_SRC 0x02 /* TX holding register empty */
#define CH943X_IIR_RDI_SRC 0x04 /* RX data interrupt */
#define CH943X_IIR_RLSE_SRC 0x06 /* RX line status error */
#define CH943X_IIR_RTOI_SRC 0x0c /* RX time-out interrupt */
#define CH943X_IIR_MSI_SRC 0x00 /* Modem status interrupt */

/* LCR register bits */
#define CH943X_LCR_LENGTH0_BIT (1 << 0) /* Word length bit 0 */

/* Word length bit 1
 * Word length bits table:
 * 00 -> 5 bit words
 * 01 -> 6 bit words
 * 10 -> 7 bit words
 * 11 -> 8 bit words
 */
#define CH943X_LCR_LENGTH1_BIT (1 << 1)

/* STOP length bit
 * STOP length bit table:
 * 0 -> 1 stop bit
 * 1 -> 1-1.5 stop bits if
 *      word length is 5,
 *      2 stop bits otherwise
 */
#define CH943X_LCR_STOPLEN_BIT (1 << 2)

#define CH943X_LCR_PARITY_BIT (1 << 3) /* Parity bit enable */
#define CH943X_LCR_ODDPARITY_BIT (0) /* Odd parity bit enable */
#define CH943X_LCR_EVENPARITY_BIT (1 << 4) /* Even parity bit enable */
#define CH943X_LCR_MARKPARITY_BIT (1 << 5) /* Mark parity bit enable */
#define CH943X_LCR_SPACEPARITY_BIT (3 << 4) /* Space parity bit enable */

#define CH943X_LCR_TXBREAK_BIT (1 << 6) /* TX break enable */
#define CH943X_LCR_DLAB_BIT (1 << 7) /* Divisor Latch enable */
#define CH943X_LCR_WORD_LEN_5 (0x00)
#define CH943X_LCR_WORD_LEN_6 (0x01)
#define CH943X_LCR_WORD_LEN_7 (0x02)
#define CH943X_LCR_WORD_LEN_8 (0x03)
#define CH943X_LCR_CONF_MODE_A CH943X_LCR_DLAB_BIT /* Special reg set */

/* MCR register bits */
#define CH943X_MCR_DTR_BIT (1 << 0) /* DTR complement */
#define CH943X_MCR_RTS_BIT (1 << 1) /* RTS complement */
#define CH943X_MCR_OUT1 (1 << 2) /* OUT1 */
#define CH943X_MCR_OUT2 (1 << 3) /* OUT2 */
#define CH943X_MCR_LOOP_BIT (1 << 4) /* Enable loopback test mode */
#define CH943X_MCR_AFE (1 << 5) /* Enable Hardware Flow control */

/* LSR register bits */
#define CH943X_LSR_DR_BIT (1 << 0) /* Receiver data ready */
#define CH943X_LSR_OE_BIT (1 << 1) /* Overrun Error */
#define CH943X_LSR_PE_BIT (1 << 2) /* Parity Error */
#define CH943X_LSR_FE_BIT (1 << 3) /* Frame Error */
#define CH943X_LSR_BI_BIT (1 << 4) /* Break Interrupt */
#define CH943X_LSR_BRK_ERROR_MASK 0x1E /* BI, FE, PE, OE bits */
#define CH943X_LSR_THRE_BIT (1 << 5) /* TX holding register empty */
#define CH943X_LSR_TEMT_BIT (1 << 6) /* Transmitter empty */
#define CH943X_LSR_FIFOE_BIT (1 << 7) /* Fifo Error */

/* MSR register bits */
#define CH943X_MSR_DCTS_BIT (1 << 0) /* Delta CTS Clear To Send */
#define CH943X_MSR_DDSR_BIT (1 << 1) /* Delta DSR Data Set Ready */
#define CH943X_MSR_DRI_BIT (1 << 2) /* Delta RI Ring Indicator */
#define CH943X_MSR_DCD_BIT (1 << 3) /* Delta CD Carrier Detect */
#define CH943X_MSR_CTS_BIT (1 << 4) /* CTS */
#define CH943X_MSR_DSR_BIT (1 << 5) /* DSR */
#define CH943X_MSR_RI_BIT (1 << 6) /* RI */
#define CH943X_MSR_CD_BIT (1 << 7) /* CD */
#define CH943X_MSR_DELTA_MASK 0x0F /* Any of the delta bits! */

/* Clock Set */
#define CH943X_CLK_PLL_BIT (1 << 7) /* PLL Enable */
#define CH943X_CLK_EXT_BIT (1 << 6) /* Extenal Clock Enable */

/* FIFO */
#define CH943X_FIFO_RD_BIT (0 << 4) /* Receive FIFO */
#define CH943X_FIFO_WR_BIT (1 << 4) /* Receive FIFO */

/* SPI Cont Mode Set */
#define CH943X_SPI_CONTE_BIT (1 << 0) /* SPI Cont Enable */

/* Misc definitions */
#define CH943X_FIFO_SIZE (1536)
#define CH943X_CMD_DELAY 3
#define CH943X_CAN_CMD_DELAY 2

#define CH9434D_GPIO_NUMS 4
#define CH9434A_GPIO_NUMS 25

#define CH943X_MAX_NUM 16
#define CH943X_GPIODRV_NAME "ch943x_io"

/* IO Command */
#define CH9434D_IO_MULTI_W_EN 0x01
#define CH9434D_IO_MULTI_R_EN 0x81
#define CH9434D_IO_DEF_W_EN 0x03
#define CH9434D_IO_DEF_R_EN 0x83

#define CH9434D_DEF_U1_ADD 1
#define CH9434D_DEF_U2_ADD 2
#define CH9434D_DEF_U3_ADD 3
#define CH9434D_DEF_U4_ADD 4
#define CH9434D_DEF_HSE_ADD 5
#define CH9434D_DEF_CAN_ADD 6
#define CH9434D_DEF_CTS0_ADD 8
#define CH9434D_DEF_CTS3_ADD 9

#define CH9434D_MUL_INT_ADD 1
#define CH9434D_MUL_TNOW0_ADD 2
#define CH9434D_MUL_TNOW1_ADD 3
#define CH9434D_MUL_TNOW2_ADD 4
#define CH9434D_MUL_TNOW3_ADD 5
#define CH9434D_MUL_RTS0_ADD 8
#define CH9434D_MUL_RTS3_ADD 11
#define CH9434D_MUL_GPIO0_ADD 12
#define CH9434D_MUL_GPIO1_ADD 13
#define CH9434D_MUL_GPIO2_ADD 14
#define CH9434D_MUL_GPIO3_ADD 15
#define CH9434D_MUL_GPIO4_ADD 16

#define CH9434D_IO_CMD_ACT 0xA5
#define CH9434D_IO_CMD_COMP 0x5A

#define CH9434_IO_SEL_FUN_CFG 0x45
#define CH9434_CAN_REG 0x46

#define CH9434_GPIO_FUNC_EN_0 0x50
#define CH9434_GPIO_FUNC_EN_1 0x51
#define CH9434_GPIO_FUNC_EN_2 0x52
#define CH9434_GPIO_FUNC_EN_3 0x53

#define CH9434_GPIO_DIR_MOD_0 0x54
#define CH9434_GPIO_DIR_MOD_1 0x55
#define CH9434_GPIO_DIR_MOD_2 0x56
#define CH9434_GPIO_DIR_MOD_3 0x57

#define CH9434_GPIO_PU_MOD_0 0x58
#define CH9434_GPIO_DIR_MOD_4 0x58
#define CH9434_GPIO_PU_MOD_1 0x59
#define CH9434_GPIO_DIR_MOD_5 0x59
#define CH9434_GPIO_PU_MOD_2 0x5A
#define CH9434_GPIO_DIR_MOD_6 0x5A
#define CH9434_GPIO_PU_MOD_3 0x5B
#define CH9434_GPIO_DIR_MOD_7 0x5B

#define CH9434_GPIO_PD_MOD_0 0x5C
#define CH9434_GPIO_SET_0 0x5C
#define CH9434_GPIO_PD_MOD_1 0x5D
#define CH9434_GPIO_SET_1 0x5D
#define CH9434_GPIO_PD_MOD_2 0x5E
#define CH9434_GPIO_RESET_0 0x5E
#define CH9434_GPIO_PD_MOD_3 0x5F
#define CH9434_GPIO_RESET_1 0x5F

#define CH9434_GPIO_PIN_VAL_0 0x60
#define CH9434_GPIO_PIN_VAL_1 0x61
#define CH9434_GPIO_PIN_VAL_2 0x62
#define CH9434_GPIO_PIN_VAL_3 0x63

/* -----------------------------------------------------------------------------
 *                         CH9434D CAN Related
 * -----------------------------------------------------------------------------
 */
#define CH9434D_CAN_CTLR 0x00
/*******************  Bit definition for CAN_CTLR register  ********************/
#define CAN_CTLR_INRQ BIT(0) /* Initialization Request */
#define CAN_CTLR_SLEEP BIT(1) /* Sleep Mode Request */
#define CAN_CTLR_TXFP BIT(2) /* Transmit FIFO Priority */
#define CAN_CTLR_RFLM BIT(3) /* Receive FIFO Locked Mode */
#define CAN_CTLR_NART BIT(4) /* No Automatic Retransmission */
#define CAN_CTLR_AWUM BIT(5) /* Automatic Wakeup Mode */
#define CAN_CTLR_ABOM BIT(6) /* Automatic Bus-Off Management */
#define CAN_CTLR_TTCM BIT(7) /* Time Triggered Communication Mode */
#define CAN_CTLR_RESET BIT(15) /* CAN software master reset */
#define CAN_CTLR_DBF \
	BIT(16) /* CAN controller operating state selection during debugging */

#define CH9434D_CAN_STATR 0x01
/*******************  Bit definition for CAN_STATR register  ********************/
#define CAN_STATR_INAK BIT(0) /* Initialization Acknowledge */
#define CAN_STATR_SLAK BIT(1) /* Sleep Acknowledge */
#define CAN_STATR_ERRI BIT(2) /* Error Interrupt */
#define CAN_STATR_WKUI BIT(3) /* Wakeup Interrupt */
#define CAN_STATR_SLAKI BIT(4) /* Sleep Acknowledge Interrupt */
#define CAN_STATR_TXM BIT(8) /* Transmit Mode */
#define CAN_STATR_RXM BIT(9) /* Receive Mode */
#define CAN_STATR_SAMP BIT(10) /* Last Sample Point */
#define CAN_STATR_RX BIT(11) /* CAN Rx Signal */

#define CH9434D_CAN_TSTATR 0x02
/*******************  Bit definition for CAN_TSTATR register  ********************/
#define CAN_TSTATR_RQCP0 BIT(0) /* Request Completed Mailbox0 */
#define CAN_TSTATR_TXOK0 BIT(1) /* Transmission OK of Mailbox0 */
#define CAN_TSTATR_ALST0 BIT(2) /* Arbitration Lost for Mailbox0 */
#define CAN_TSTATR_TERR0 BIT(3) /* Transmission Error of Mailbox0 */
#define CAN_TSTATR_ABRQ0 BIT(7) /* Abort Request for Mailbox0 */
#define CAN_TSTATR_RQCP1 BIT(8) /* Request Completed Mailbox1 */
#define CAN_TSTATR_TXOK1 BIT(9) /* Transmission OK of Mailbox1 */
#define CAN_TSTATR_ALST1 BIT(10) /* Arbitration Lost for Mailbox1 */
#define CAN_TSTATR_TERR1 BIT(11) /* Transmission Error of Mailbox1 */
#define CAN_TSTATR_ABRQ1 BIT(15) /* Abort Request for Mailbox 1 */
#define CAN_TSTATR_RQCP2 BIT(16) /* Request Completed Mailbox2 */
#define CAN_TSTATR_TXOK2 BIT(17) /* Transmission OK of Mailbox 2 */
#define CAN_TSTATR_ALST2 BIT(18) /* Arbitration Lost for mailbox 2 */
#define CAN_TSTATR_TERR2 BIT(19) /* Transmission Error of Mailbox 2 */
#define CAN_TSTATR_ABRQ2 BIT(23) /* Abort Request for Mailbox 2 */

#define CAN_TSTATR_TME0 BIT(26) /* Transmit Mailbox 0 Empty */
#define CAN_TSTATR_TME1 BIT(27) /* Transmit Mailbox 1 Empty */
#define CAN_TSTATR_TME2 BIT(28) /* Transmit Mailbox 2 Empty */

#define CH9434D_CAN_RFIFO0 0x03
#define CH9434D_CAN_RFIFO1 0x04
/*******************  Bit definition for CAN_RFIFOx register  *******************/
#define CAN_RFIFOx_FMPx ((u32)0x000000FF) /* FIFO x Message Pending */
#define CAN_RFIFOx_FULLx BIT(16) /* FIFO x Full */
#define CAN_RFIFOx_FOVRx BIT(17) /* FIFO x Overrun */
#define CAN_RFIFOx_RFOMx BIT(18) /* Release FIFO x Output Mailbox */

#define CH9434D_CAN_INTENR 0x05
/********************  Bit definition for CAN_INTENR register  *******************/
#define CAN_INTENR_TMEIE \
	BIT(0) /* Transmit Mailbox Empty Interrupt Enable */
#define CAN_INTENR_FMPIE0 \
	BIT(1) /* FIFO Message Pending Interrupt Enable */
#define CAN_INTENR_FFIE0 BIT(2) /* FIFO Full Interrupt Enable */
#define CAN_INTENR_FOVIE0 BIT(3) /* FIFO Overrun Interrupt Enable */
#define CAN_INTENR_FMPIE1 \
	BIT(4) /* FIFO Message Pending Interrupt Enable */
#define CAN_INTENR_FFIE1 BIT(5) /* FIFO Full Interrupt Enable */
#define CAN_INTENR_FOVIE1 BIT(6) /* FIFO Overrun Interrupt Enable */
#define CAN_INTENR_EWGIE BIT(8) /* Error Warning Interrupt Enable */
#define CAN_INTENR_EPVIE BIT(9) /* Error Passive Interrupt Enable */
#define CAN_INTENR_BOFIE BIT(10) /* Bus-Off Interrupt Enable */
#define CAN_INTENR_LECIE BIT(11) /* Last Error Code Interrupt Enable */
#define CAN_INTENR_ERRIE BIT(15) /* Error Interrupt Enable */
#define CAN_INTENR_WKUIE BIT(16) /* Wakeup Interrupt Enable */
#define CAN_INTENR_SLKIE BIT(17) /* Sleep Interrupt Enable */

#define CH9434D_CAN_ERRSR 0x06
/********************  Bit definition for CAN_ERRSR register  *******************/
#define CAN_ERRSR_EWGF BIT(0) /* Error Warning Flag */
#define CAN_ERRSR_EPVF BIT(1) /* Error Passive Flag */
#define CAN_ERRSR_BOFF BIT(2) /* Bus-Off Flag */

#define CAN_ERRSR_TEC \
	((u32)0x00FF0000) /* Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ERRSR_REC ((u32)0xFF000000) /* Receive Error Counter */

#define CH9434D_CAN_BTIMR 0x07
/*******************  Bit definition for CAN_BTIMR register  ********************/
#define CAN_BTIMR_BRP ((u32)0x000003FF) /* Baud Rate Prescaler */
#define CAN_BTIMR_TS1 ((u32)0x000F0000) /* Time Segment 1 */
#define CAN_BTIMR_TS2 ((u32)0x00700000) /* Time Segment 2 */
#define CAN_BTIMR_SJW ((u32)0x03000000) /* Resynchronization Jump Width */
#define CAN_BTIMR_LBKM BIT(30) /* Loop Back Mode (Debug) */
#define CAN_BTIMR_SILM BIT(31) /* Silent Mode */

#define CH9434D_CAN_TTCTLR 0x08
/********************  Bit definition for CAN_TTCTLR register  *******************/
#define CAN_TTCTLR_TIMCMV ((u32)0x0000FFFF)
#define CAN_TTCTLR_TIMRST ((u32)0x00010000)
#define CAN_TTCTLR_MODE ((u32)0x00020000)

#define CH9434D_CAN_TTCNT 0x09
/********************  Bit definition for CAN_TTCNT register  *******************/
#define CAN_TTCNT ((u32)0x0000FFFF)

#define CH9434D_CAN_TERR_CNT 0x0A
/********************  Bit definition for CAN_TERR_CNT register  *******************/
#define CAN_TERR_CNT ((u32)0x000001FF)

#define CH9434D_CAN_TXMIR0 0x0B
/******************  Bit definition for CAN_TXMI0R register  ********************/
#define CAN_TXMIRx_TXRQ ((u32)0x00000001) /* Transmit Mailbox Request */
#define CAN_TXMIRx_RTR ((u32)0x00000002) /* Remote Transmission Request */
#define CAN_TXMIRx_IDE ((u32)0x00000004) /* Identifier Extension */
#define CAN_TXMIRx_EXID ((u32)0x001FFFF8) /* Extended Identifier */
#define CAN_TXMIRx_STID \
	((u32)0xFFE00000) /* Standard Identifier or Extended Identifier */

#define CH9434D_CAN_TXMDTR0 0x0C
/******************  Bit definition for CAN_TXMDT0R register  *******************/
#define CAN_TXMDTRx_DLC ((u32)0x0000000F) /* Data Length Code */
#define CAN_TXMDTRx_TGT ((u32)0x00000100) /* Transmit Global Time */
#define CAN_TXMDTRx_TIME ((u32)0xFFFF0000) /* Message Time Stamp */

#define CH9434D_CAN_TXMDLR0 0x0D
/******************  Bit definition for CAN_TXMDL0R register  *******************/
#define CAN_TXMDLRx_DATA0 ((u32)0x000000FF) /* Data byte 0 */
#define CAN_TXMDLRx_DATA1 ((u32)0x0000FF00) /* Data byte 1 */
#define CAN_TXMDLRx_DATA2 ((u32)0x00FF0000) /* Data byte 2 */
#define CAN_TXMDLRx_DATA3 ((u32)0xFF000000) /* Data byte 3 */

#define CH9434D_CAN_TXMDHR0 0x0E
/******************  Bit definition for CAN_TXMDH0R register  *******************/
#define CAN_TXMDHRx_DATA4 ((u32)0x000000FF) /* Data byte 4 */
#define CAN_TXMDHRx_DATA5 ((u32)0x0000FF00) /* Data byte 5 */
#define CAN_TXMDHRx_DATA6 ((u32)0x00FF0000) /* Data byte 6 */
#define CAN_TXMDHRx_DATA7 ((u32)0xFF000000) /* Data byte 7 */

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
#define CAN_RXMIRx_RTR ((u32)0x00000002) /* Remote Transmission Request */
#define CAN_RXMIRx_IDE ((u32)0x00000004) /* Identifier Extension */
#define CAN_RXMIRx_EXID ((u32)0x001FFFF8) /* Extended Identifier */
#define CAN_RXMIRx_STID \
	((u32)0xFFE00000) /* Standard Identifier or Extended Identifier */

#define CH9434D_CAN_RXMDTR0 0x18
/*******************  Bit definition for CAN_RXMDT0R register  ******************/
#define CAN_RXMDTRx_DLC ((u32)0x0000000F) /* Data Length Code */
#define CAN_RXMDTRx_BRS ((u32)0x00000010)
#define CAN_RXMDTRx_ESI ((u32)0x00000020)
#define CAN_RXMDHRx_RES ((u32)0x00000100)
#define CAN_RXMDTRx_FMI ((u32)0x0000FF00) /* Filter Match Index */
#define CAN_RXMDTRx_TIME ((u32)0xFFFF0000) /* Message Time Stamp */

#define CH9434D_CAN_RXMDLR0 0x19
/*******************  Bit definition for CAN_RXMDL0R register  ******************/
#define CAN_RXMDLRx_DATA0 ((u32)0x000000FF) /* Data byte 0 */
#define CAN_RXMDLRx_DATA1 ((u32)0x0000FF00) /* Data byte 1 */
#define CAN_RXMDLRx_DATA2 ((u32)0x00FF0000) /* Data byte 2 */
#define CAN_RXMDLRx_DATA3 ((u32)0xFF000000) /* Data byte 3 */

#define CH9434D_CAN_RXMDHR0 0x1A
/*******************  Bit definition for CAN_RXMDH0R register  ******************/
#define CAN_RXMDHRx_DATA4 ((u32)0x000000FF) /* Data byte 4 */
#define CAN_RXMDHRx_DATA5 ((u32)0x0000FF00) /* Data byte 5 */
#define CAN_RXMDHRx_DATA6 ((u32)0x00FF0000) /* Data byte 6 */
#define CAN_RXMDHRx_DATA7 ((u32)0xFF000000) /* Data byte 7 */

#define CH9434D_CAN_RX0READ_CONT 0x40
#define CH9434D_CAN_RX1READ_CONT 0x41
#define CH9434D_CAN_TX0WRITE_CONT 0x42
#define CH9434D_CAN_TX1WRITE_CONT 0x43
#define CH9434D_CAN_TX2WRITE_CONT 0x44

#define CH9434D_CAN_RXMIR1 0x1B
#define CH9434D_CAN_RXMDTR1 0x1C
#define CH9434D_CAN_RXMDLR1 0x1D
#define CH9434D_CAN_RXMDHR1 0x1E

#define CH9434D_CAN_FCTLR 0x1F
/*******************  Bit definition for CAN_FCTLR register  ********************/
#define CAN_FCTLR_FINIT ((uint8_t)BIT(0)) /* Filter Init Mode */

#define CH9434D_CAN_FMCFGR 0x20
/*******************  Bit definition for CAN_FMCFGR register  *******************/
#define CAN_FMCFGR_FBM_MASK ((u32)0x00003FFF) /* Filter Mode */
#define CAN_FMCFGR_FBM(x) ((u32)(1 << x)) /* Filter Init Mode bit x */

#define CH9434D_CAN_FSCFGR 0x21
/*******************  Bit definition for CAN_FSCFGR register  *******************/
#define CAN_FSCFGR_FSC_MASK \
	((u32)0x00003FFF) /* Filter Scale Configuration */
#define CAN_FSCFGR_FSC(x) \
	((u32)(1 << x)) /* Filter Scale Configuration bit x */

#define CH9434D_CAN_FAFIFOR 0x22
/******************  Bit definition for CAN_FAFIFOR register  *******************/
#define CAN_FAFIFOR_FFA_MASK ((u32)0x00003FFF) /* Filter FIFO Assignment */
#define CAN_FAFIFOR_FFA(x) \
	((u32)(1 << x)) /* Filter FIFO Assignment for Filter x */

#define CH9434D_CAN_FWR 0x23
/*******************  Bit definition for CAN_FWR register  *******************/
#define CAN_FWR_FACT_MASK ((u32)0x00003FFF) /* Filter Active */
#define CAN_FWR_FACT(x) ((u32)(1 << x)) /* Filter x Active */

/*******************  Bit definition for CAN_FnRm register  *******************/
#define CAN_FnRm_FB(x) ((u32)(1 << x)) /* Filter bit x */
#define CH9434D_CAN_FxR1(x) (0x24 + x * 2)
#define CH9434D_CAN_FxR2(x) (0x25 + x * 2)

#define CH943X_DELAY_MS (5)

/* CAN_identifier_type */
#define CAN_Id_Extended BIT(2) /* Extended Id */

/* CAN_transmit_constants */
#define CAN_TxStatus_Failed ((u8)0x00) /* CAN transmission failed */
#define CAN_TxStatus_Ok ((u8)0x01) /* CAN transmission succeeded */
#define CAN_TxStatus_Pending ((u8)0x02) /* CAN transmission pending */
#define CAN_TxStatus_NoMailBox \
	((u8)0x04) /* CAN cell did not provide an empty mailbox */

#define to_ch943x_one(p, e) ((container_of((p), struct ch943x_one, e)))

#define REGS_BUFSIZE 1024

struct ch943x_devtype {
	char name[10];
	int nr_uart;
};

enum CHIPTYPE {
	CHIP_CH9434A = 0,
	CHIP_CH9434D,
	CHIP_CH9434M,
};

struct ch943x_port;

struct ch943x_can_priv {
	struct can_priv can;
	struct net_device *ndev;
	struct spi_device *spi;
	struct ch943x_port *s;
	struct mutex reg_lock;
	struct mutex ops_lock;
	struct mutex can_lock;
	struct sk_buff *tx_skb;
	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct restart_work;
	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
#define AFTER_SUSPEND_POWER 4
#define AFTER_SUSPEND_RESTART 8
	int restart_tx;
	bool tx_busy;
	atomic_t can_isopen;
	struct regulator *power;
	struct regulator *transceiver;
};

struct ch943x_one {
	struct uart_port port;
	struct work_struct tx_work;
	struct work_struct md_work;
	struct work_struct stop_rx_work;
	struct work_struct stop_tx_work;
	struct serial_rs485 rs485;
	unsigned char msr_reg;
	unsigned char ier;
	unsigned char mcr_force;
	atomic_t isopen;
	unsigned char *txbuf;
	unsigned char *rxbuf;
};

struct ch943x_port {
	struct list_head ch943x_list;
	struct uart_driver uart;
	struct ch943x_devtype *devtype;
	struct ch943x_can_priv *priv;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
#endif
	struct mutex mutex;
	struct mutex mutex_bus_access;
	struct clk *clk;
#ifdef USE_SPI_MODE
	struct spi_device *spi_dev;
#else
	struct i2c_client *i2c;
#endif
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
	struct dentry *debugfile;
#endif
	struct device *dev;
	uint8_t reg485;
	int irq;
	bool spi_contmode;
	enum CHIPTYPE chiptype;
	bool use_uart_flag;
	bool use_can_flag;

	struct class *ch943x_io_class;
	struct cdev cdev;
	dev_t devt;

	uint8_t ver[VER_LEN];
	uint8_t gpio_pinval_bits[2];
	uint8_t gpio_en_bits[8];
	uint8_t gpio_mod_bits[8];
	uint8_t gpio_set_val[2];
	uint8_t gpio_reset_val[2];

	uint8_t gpio_pu_mod[8];

	unsigned char buf[65536];
	struct ch943x_one p[0];
};

extern int ch943x_gpio_request(struct gpio_chip *chip, unsigned offset);
extern void ch943x_gpio_free(struct gpio_chip *chip, unsigned offset);
extern int ch943x_gpio_direction_input(struct gpio_chip *chip,
				       unsigned offset);
extern int ch943x_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value);
extern int ch943x_set_config(struct gpio_chip *chip, unsigned offset,
			     unsigned long config);
extern int ch943x_gpio_get(struct gpio_chip *chip, unsigned offset);
extern void ch943x_gpio_set(struct gpio_chip *chip, unsigned int offset,
			    int value);

extern int ch943x_iofunc_cfg(struct ch943x_port *s, uint8_t reg,
			     uint8_t io_cmd, uint8_t io_addr);
extern int ch943x_iofunc_get_status(struct ch943x_port *s, uint8_t reg,
				    uint8_t io_cmd, uint8_t io_addr,
				    uint8_t *rxbuf);
extern uint8_t ch943x_port_read(struct uart_port *port, uint8_t reg);
extern uint8_t ch943x_reg_read(struct ch943x_port *s, uint8_t reg);
extern int ch943x_port_write(struct uart_port *port, uint8_t reg,
			     uint8_t val);
extern int ch943x_reg_write(struct ch943x_port *s, uint8_t reg,
			    uint8_t val);
extern int ch943x_port_read_version(struct ch943x_port *s, uint8_t reg,
				    uint8_t *buf, uint8_t count);
extern int ch943x_port_update(struct uart_port *port, uint8_t reg,
			      uint8_t mask, uint8_t val);
extern int ch943x_raw_write(struct uart_port *port, const void *reg,
			    unsigned char *buf, int len);
extern int ch943x_raw_read(struct uart_port *port, uint8_t reg,
			   unsigned char *buf, int len);
extern int ch943x_get_chip_version(struct ch943x_port *s);
extern int ch943x_scr_test(struct uart_port *port);

extern int ch943x_register_uart_driver(struct ch943x_port *s);
extern int ch943x_register_uart_port(struct ch943x_port *s);
void ch943x_uart_remove(struct ch943x_port *s);

extern int ch943x_register_sys_gpio(struct ch943x_port *s);
extern void ch943x_sys_gpio_remove(struct ch943x_port *s);
int ch943x_gpio_iodev_init(struct ch943x_port *s);
int ch943x_gpio_iodev_exit(struct ch943x_port *s);

extern int ch943x_can_resgister(struct ch943x_port *s);
extern void ch943x_can_remove(struct ch943x_port *s);

extern irqreturn_t ch943x_ist_top(int irq, void *dev_id);
extern irqreturn_t ch943x_ist(int irq, void *dev_id);

extern void ch943x_can_irq(struct ch943x_port *s);
extern void ch943x_port_irq(struct ch943x_port *s, int portno);

extern int ch943x_debugfs_init(struct ch943x_port *s);
extern void ch943x_debugfs_exit(struct ch943x_port *s);

extern int ch943x_canreg_write(struct ch943x_port *s, u8 reg, u32 val);
extern u32 ch943x_canreg_read(struct ch943x_port *s, u8 reg);
extern int ch943x_canreg_contmode_read(struct ch943x_port *s, u8 reg,
				       u8 *rxbuf);
extern int ch943x_canreg_contmode_write(struct ch943x_port *s, u8 reg,
					u8 *txbuf, u8 txlen);
extern int ch943x_ctrl_write(struct ch943x_port *s, u8 cmd, u32 datalen,
			     void *data);
extern int ch943x_ctrl_read(struct ch943x_port *s, u8 cmd, u32 datalen,
			    void *data);
