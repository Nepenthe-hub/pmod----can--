/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH9434.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : SPI to Serial Port Chip CH9434 Operating Interface
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
/* 原始: #include "debug.h"  — CH32F10x 平台头文件，STM32F103 上不需要 */
/* CH9434.h 内部已自定义 u8_t/u16_t/u32_t，且外部函数通过 CH9434_Port.c 实现 */
#include "CH9434.h"

/*
CH9434A:
1. Chip clock configuration related notes:
1. External crystal oscillator: 32M
2. Internal clock frequency: 32M
3. Enable multiplier coefficient: 15 (fixed)
4. The maximum clock frequency set for the chip should not exceed 40M
5. Serial port reference clock: Without multiplier: 32MHz  With multiplier: 32MHz * 15 / Division factor

II. Recommended Method for Calculating Common Baud Rates
1. 32M -> Calculation of baud rate using serial port reference clock: 32M / 8 / baud rate
2. 32 * 15 / 13 = 36.923M -> Calculation of baud rate using serial port reference clock: 36.923M / 8 / baud rate (e.g. baud rate: 921600)

III. Variable Storage Definitions
1. osc_xt_frequency: Records the frequency of the external crystal oscillator. It is recorded when using an external crystal oscillator and can be modified by calling CH9434OscXtFreqSet.
2. sys_frequency: Based on the configured clock mode, it calculates the CH9434 serial port reference clock, which is used for subsequent baud rate calculations.
3. lower_power_reg: Records the low-power state of CH9434.
4. ch9434_gpio_x_val: The output level value of the CH9434 general-purpose GPIO. It is defined bit by bit and the function for controlling the pin level is: CH9434GPIOPinOut.


CH9434D:
1. Chip clock configuration description:
1. External crystal oscillator: 16M
2. Internal clock frequency: 96M
3. Multiplication factor: Fixed at 6, the chip will multiply the frequency to 96M
4. Serial port reference clock: Fixed at 96M
5. Clock configuration is not required, only switching is needed

II. Calculation Method of Baud Rate
1. DLM and DLL form a 16-bit register DL. The high 12 bits represent the integer part of the divisor, and the low 4 bits represent the fractional part of the divisor.
2. The integer part DIV_M value is: DL[15:4], and the fractional part DIV_F value is: DL[3:0] / 16.
3. The serial port reference clock is: 96M.
4. Calculate the baud rate: 96M / (16 * (DIV_M + DIV_F / 16))

III. Interface Related
1. CH9434D needs to first set the communication interface through the API. The program defaults to using the SPI interface, @CH9434DSetIf;
2. IIC requires setting the IIC address, @CH9434IICAddSet;

*/

// Define the frequency of the external crystal oscillator
u32_t osc_xt_frequency = 32000000;

// Define the current serial port reference clock frequency
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
u32_t sys_frequency = 32000000;  // The chip is set by default to have an internal capacity of 32M.
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
u32_t sys_frequency = 96000000;  // The chip is set by default to have an internal capacity of 96M.
#endif
// Sleep Mode
u8_t lower_power_reg = 0;

// The output level values of GPIO, totaling 24 GPIOs.
u32_t ch9434_gpio_x_val = 0;

// Address byte of IIC
u8_t ch9434d_iic_add = 0;

/*******************************************************************************
 * Function Name  : CH9434IICAddSet
 * Description    : CH9434 IIC Interface Address Setting
 * Input          : set_iic_add: The IIC address set by hardware, the address value for pin setting: 0 - 3
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434IICAddSet (u8_t set_iic_add) {
    ch9434d_iic_add = (CH9434D_IIC_ADD_DEF | (set_iic_add % 4)) << 1;
}

/*******************************************************************************
 * Function Name  : CH9434OscXtFreqSet
 * Description    : External crystal oscillator frequency recording.
 * Input          : x_freq:The current frequency of the crystal oscillator connected to the chip.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434OscXtFreqSet (u32_t x_freq) {
    osc_xt_frequency = x_freq;
}

/*******************************************************************************
 * Function Name  : CH9434RegWriteOneByte
 * Description    : CH9434 main control write register - one byte
 * Input          : reg_ad: Register Address; reg_val: Value to be Written
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434RegWriteOneByte (u8_t reg_ad, u8_t reg_val) {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_WRITE | reg_ad);
    CH9434_US_DELAY();
    CH9434_SPI_WRITE_BYTE (reg_val);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_WRITE | reg_ad);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_WRITE_BYTE (reg_val);
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
#endif
#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_WRITE | reg_ad);
    CH9434_IIC_SEND_DATA (reg_val);
    CH9434_IIC_STOP();
#endif
#endif
}

/*******************************************************************************
 * Function Name  : CH9434RegReadOneByte
 * Description    : CH9434 main control reads one byte of the register
 * Input          : reg_ad: Register Address
 * Output         : None
 * Return         : The current value of the register
 *******************************************************************************/
u8_t CH9434RegReadOneByte (u8_t reg_ad) {
    u8_t reg_val = 0;

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_READ | reg_ad);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    reg_val = CH9434_SPI_WRITE_BYTE (0xff);
    CH9434_US_DELAY();
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_READ | reg_ad);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    reg_val = CH9434_SPI_WRITE_BYTE (0xff);
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
#endif
#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_READ | reg_ad);

    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_R | ch9434d_iic_add);
    reg_val = CH9434_IIC_READ_DATA();
    CH9434_IIC_STOP();
#endif
#endif

    return reg_val;
}

/*******************************************************************************
 * Function Name  : CH9434RegWriteBytes
 * Description    : CH9434 main control write register for multiple bytes
 * Input          : reg_ad: Register Address
                    p_data: Cache address;
                    len: Length
 * Output         : None
 * Return         : None
 *******************************************************************************/
u8_t CH9434RegWriteBytes (u8_t reg_ad, u8_t *p_data, u16_t len) {
    u16_t i;

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    for (i = 0; i < len; i++) {
        CH9434_SPI_SCS_OP (CH9434_DISABLE);
        CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_WRITE | reg_ad);
        CH9434_US_DELAY();
        CH9434_SPI_WRITE_BYTE (p_data[i]);
        CH9434_US_DELAY();
        CH9434_US_DELAY();
        CH9434_US_DELAY();
        CH9434_SPI_SCS_OP (CH9434_ENABLE);
    }
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_WRITE | reg_ad);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    for (i = 0; i < len; i++) CH9434_SPI_WRITE_BYTE (p_data[i]);
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
#endif
#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_WRITE | reg_ad);
    for (i = 0; i < len; i++) CH9434_IIC_SEND_DATA (p_data[i]);
    CH9434_IIC_STOP();

    

#endif
#endif
    return 0;
}

/*******************************************************************************
 * Function Name  : CH9434RegReadBytes
 * Description    : CH9434 main control reads multiple bytes of the register
 * Input          : reg_ad: Register Address
 * Output         : None
 * Return         : The current value of the register
 *******************************************************************************/
u8_t CH9434RegReadBytes (u8_t reg_ad, u8_t *p_data, u16_t len) {
    // u8_t reg_val = 0;
    u16_t i;

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    for (i = 0; i < len; i++) {
        CH9434_SPI_SCS_OP (CH9434_DISABLE);
        CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_READ | reg_ad);
        CH9434_US_DELAY();
        CH9434_US_DELAY();
        CH9434_US_DELAY();
        p_data[i] = CH9434_SPI_WRITE_BYTE (0xff);
        CH9434_US_DELAY();
        CH9434_SPI_SCS_OP (CH9434_ENABLE);
    }
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_READ | reg_ad);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    for (i = 0; i < len; i++) p_data[i] = CH9434_SPI_WRITE_BYTE (0xff);
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
#endif
#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_READ | reg_ad);
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_R | ch9434d_iic_add);
    for (i = 0; i < len; i++) p_data[i] = CH9434_IIC_READ_DATA();
    CH9434_IIC_STOP();
#endif
#endif
    return 0;
}

/*******************************************************************************
* Function Name  : CH9434InitClkMode
* Description    : CH9434 Chip Clock Mode Configuration
* Input          : xt_en: External Oscillator Enable
                   freq_mul_en: Frequency Multiplication Function Enable
                   div_num: Division Coefficient
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434InitClkMode (u8_t xt_en, u8_t freq_mul_en, u8_t div_num) {
    u8_t clk_ctrl_reg;
    u16_t i;

    clk_ctrl_reg = 0;
    if (freq_mul_en)
        clk_ctrl_reg |= (1 << 7);
    if (xt_en)
        clk_ctrl_reg |= (1 << 6);
    clk_ctrl_reg |= (div_num & 0x1f);

    /* Calculate the current serial port reference clock */
 
    switch (clk_ctrl_reg & 0xc0) {
    case 0x00:  
    {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
        sys_frequency = 32000000;
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
        sys_frequency = 96000000;
#endif
        break;
    }
    case 0x40: 
    {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
        if ((osc_xt_frequency > 36000000) || (osc_xt_frequency < 24000000)) 
        {
            return;
        }
        sys_frequency = osc_xt_frequency;
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
        sys_frequency = 96000000;
#endif
        break;
    }
    case 0x80: 
    {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
        sys_frequency = 480000000 / (div_num & 0x1f);
        if (sys_frequency > 40000000)  
        {
            sys_frequency = 32000000;
            return;
        }
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
        sys_frequency = 96000000;
#endif
        break;
    }
    case 0xc0:  
    {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
        if ((osc_xt_frequency > 36000000) || (osc_xt_frequency < 24000000))  
        {
            return;
        }
        sys_frequency = osc_xt_frequency * 15 / (div_num & 0x1f);
        if (sys_frequency > 40000000) 
        {
            sys_frequency = 32000000;
            return;
        }
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
        sys_frequency = 96000000;
#endif
        break;
    }
    }

    CH9434RegWriteOneByte (CH9434_CLK_CTRL_CFG_ADD, clk_ctrl_reg);
    for (i = 0; i < 50000; i++) CH9434_US_DELAY();  
}

/*******************************************************************************
* Function Name  : CH9434UARTxParaSet
* Description    : Serial port parameter settings
* Input          : uart_idx: Serial port number
                   bps: Serial port's baud rate
                   data_bits: Data bits
                   stop_bits: Stop bits
                   veri_bits: Check bits
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxParaSet (u8_t uart_idx, u32_t bps, u8_t data_bits, u8_t stop_bits, u8_t veri_bits) {
    u8_t uart_reg_dll;
    u8_t uart_reg_dlm;
    u32_t x;
    u8_t uart_reg_lcr;

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    x = 10 * sys_frequency / 8 / bps;
    x = (x + 5) / 10;

    uart_reg_dll = x & 0xff;
    uart_reg_dlm = (x >> 8) & 0xff;
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
    u32_t integerdivider = 0x00;
    u32_t fractionaldivider = 0x00;

    integerdivider = ((25 * sys_frequency) / (4 * bps));
    x = (integerdivider / 100) << 4;
    fractionaldivider = integerdivider - (100 * (x >> 4));
	fractionaldivider = ((((fractionaldivider * 16) + 50) / 100));
    x |= fractionaldivider & ((u8_t)0x0F);

	if(fractionaldivider & 0x10)
	{
		if((x >> 4) == 0xfff)
		{
			x |= 0x0f;
		}
		else
		{
			x += (1<<4);
		}
	} 
    uart_reg_dll = x & 0xff;
    uart_reg_dlm = (x >> 8) & 0xff;


#endif

    uart_reg_lcr = CH9434RegReadOneByte (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx);

    uart_reg_lcr |= CH9434_UARTx_BIT_DLAB;

    uart_reg_lcr &= ~0x03;
    switch (data_bits) {
    case CH9434_UART_5_BITS_PER_CHAR:
        break;
    case CH9434_UART_6_BITS_PER_CHAR:
        uart_reg_lcr |= 0x01;
        break;
    case CH9434_UART_7_BITS_PER_CHAR:
        uart_reg_lcr |= 0x02;
        break;
    case CH9434_UART_8_BITS_PER_CHAR:
        uart_reg_lcr |= 0x03;
        break;
    default:
        uart_reg_lcr |= 0x03;
        break;
    }

    uart_reg_lcr &= ~(1 << 2);
    if (stop_bits == CH9434_UART_TWO_STOP_BITS) {
        uart_reg_lcr |= (1 << 2);
    }

    uart_reg_lcr &= ~(1 << 3);
    uart_reg_lcr &= ~(3 << 4);
    switch (veri_bits) {
    case CH9434_UART_NO_PARITY:
        break;
    case CH9434_UART_ODD_PARITY:
        uart_reg_lcr |= (1 << 3);
        break;
    case CH9434_UART_EVEN_PARITY:
        uart_reg_lcr |= (1 << 3);
        uart_reg_lcr |= (1 << 4);
        break;
    case CH9434_UART_MARK_PARITY:
        uart_reg_lcr |= (1 << 3);
        uart_reg_lcr |= (2 << 4);
        break;
    case CH9434_UART_SPACE_PARITY:
        uart_reg_lcr |= (1 << 3);
        uart_reg_lcr |= (3 << 4);
        break;
    default:
        break;
    }

    CH9434RegWriteOneByte (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx, uart_reg_lcr);
    CH9434RegWriteOneByte (CH9434_UARTx_DLL_ADD + 0x10 * uart_idx, uart_reg_dll);
    CH9434RegWriteOneByte (CH9434_UARTx_DLM_ADD + 0x10 * uart_idx, uart_reg_dlm);

    uart_reg_lcr &= ~CH9434_UARTx_BIT_DLAB;
    CH9434RegWriteOneByte (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx, uart_reg_lcr);
}

/*******************************************************************************
* Function Name  : CH9434UARTxFIFOSet
* Description    : Serial port FIFO settings
* Input          : uart_idx: Serial port number
                   fifo_en: FIFO function enable
                   fifo_level: FIFO trigger level
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxFIFOSet (u8_t uart_idx, u8_t fifo_en, u8_t fifo_level) {
    u8_t uart_reg_fcr;

    uart_reg_fcr = 0;
    if (fifo_en) {
        uart_reg_fcr |= 0x01;
        uart_reg_fcr |= fifo_level << 6;
    }

    CH9434RegWriteOneByte (CH9434_UARTx_FCR_ADD + 0x10 * uart_idx, uart_reg_fcr);
}

/*******************************************************************************
* Function Name  : CH9434UARTxIrqSet
* Description    : Serial port interrupt settings
* Input          : uart_idx: Serial port number
                   modem: Modem signal interruption
                   line: Line status interruption
                   tx: Transmission interruption
                   rx: Reception interruption
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxIrqSet (u8_t uart_idx, u8_t modem, u8_t line, u8_t tx, u8_t rx) {
    u8_t uart_reg_ier;

    uart_reg_ier = 0;
    if (modem)
        uart_reg_ier |= (1 << 3);
    if (line)
        uart_reg_ier |= (1 << 2);
    if (tx)
        uart_reg_ier |= (1 << 1);
    if (rx)
        uart_reg_ier |= (1 << 0);

    CH9434RegWriteOneByte (CH9434_UARTx_IER_ADD + 0x10 * uart_idx, uart_reg_ier);
}


/*******************************************************************************
* Function Name  : CH9434UARTxFlowSet
* Description    : Flow control function and pin settings 
* Input          : uart_idx: Serial port number
                   flow_en: Flow Control Enable
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxFlowSet (u8_t uart_idx, u8_t flow_en) {
    u8_t uart_reg_mcr;

    uart_reg_mcr = CH9434RegReadOneByte (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx);
    uart_reg_mcr &= ~(1 << 5);
    if (flow_en)
        uart_reg_mcr |= (1 << 5);
    CH9434RegWriteOneByte (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx, uart_reg_mcr);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxIrqOpen
 * Description    : Enable the interrupt serial port request
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434UARTxIrqOpen (u8_t uart_idx) {
    u8_t uart_reg_mcr;

    uart_reg_mcr = CH9434RegReadOneByte (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx);
    uart_reg_mcr |= (1 << 3);
    CH9434RegWriteOneByte (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx, uart_reg_mcr);
}

/*******************************************************************************
* Function Name  : CH9434UARTxRtsDtrPin
* Description    : Set the RTS and DTR pins of the serial port
* Input          : uart_idx:Serial port number
                   rts_val:The level state of the RTS pin
                   dtr_val:The level state of the DTR pin
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxRtsDtrPin (u8_t uart_idx, u8_t rts_val, u8_t dtr_val) {
    u8_t uart_reg_mcr;

    uart_reg_mcr = CH9434RegReadOneByte (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx);
    if (rts_val)
        uart_reg_mcr |= (1 << 1);
    else
        uart_reg_mcr &= ~(1 << 1);

    if (dtr_val)
        uart_reg_mcr |= (1 << 0);
    else
        uart_reg_mcr &= ~(1 << 0);
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
    uart_reg_mcr &= ~(1 << 0);
#endif
    CH9434RegWriteOneByte (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx, uart_reg_mcr);
}

/*******************************************************************************
* Function Name  : CH9434UARTxWriteSRC
* Description    : SRC register write operation
* Input          : uart_idx:Serial port number
                   src_val:Value of the SRC register
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxWriteSRC (u8_t uart_idx, u8_t src_val) {
    CH9434RegWriteOneByte (CH9434_UARTx_SCR_ADD + 0x10 * uart_idx, src_val);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxReadSRC
 * Description    : SRC register read operation
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : Value of the SRC register
 *******************************************************************************/
u8_t CH9434UARTxReadSRC (u8_t uart_idx) {
    return CH9434RegReadOneByte (CH9434_UARTx_SCR_ADD + 0x10 * uart_idx);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxReadIIR
 * Description    : Serial port interrupt code query
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : IIR register value
 *******************************************************************************/
u8_t CH9434UARTxReadIIR (u8_t uart_idx) {
    return CH9434RegReadOneByte (CH9434_UARTx_IIR_ADD + 0x10 * uart_idx);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxReadLSR
 * Description    : Reading of the serial port LSR register
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : LSR register value
 *******************************************************************************/
u8_t CH9434UARTxReadLSR (u8_t uart_idx) {
    return CH9434RegReadOneByte (CH9434_UARTx_LSR_ADD + 0x10 * uart_idx);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxReadMSR
 * Description    : Serial port MSR register reading
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : MSR register value
 *******************************************************************************/
u8_t CH9434UARTxReadMSR (u8_t uart_idx) {
    return CH9434RegReadOneByte (CH9434_UARTx_MSR_ADD + 0x10 * uart_idx);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxGetRxFIFOLen
 * Description    : Obtain the length of the received data through the serial port
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : Size of the serial port receive FIFO
 *******************************************************************************/
u16_t CH9434UARTxGetRxFIFOLen (u8_t uart_idx) {
    u8_t uart_fifo_ctrl = 0;
    u8_t uart_fifo_cnt_l;
    u8_t uart_fifo_cnt_h;
    u16_t uart_fifo_cnt = 0;

    uart_fifo_ctrl |= uart_idx;
    CH9434RegWriteOneByte (CH9434_FIFO_CTRL_ADD, uart_fifo_ctrl);
    uart_fifo_cnt_l = CH9434RegReadOneByte (CH9434_FIFO_CTRL_L_ADD);
    uart_fifo_cnt_h = CH9434RegReadOneByte (CH9434_FIFO_CTRL_H_ADD);
    uart_fifo_cnt = uart_fifo_cnt_h;
    uart_fifo_cnt = (uart_fifo_cnt << 8) | uart_fifo_cnt_l;

    return uart_fifo_cnt;
}

/*******************************************************************************
* Function Name  : CH9434UARTxGetRxFIFOData
* Description    : Read the received data from the serial port
* Input          : uart_idx:Serial port number
                   p_data:Data storage pointer
                   read_len:The length of the read data
* Output         : None
* Return         : The current value of the register
*******************************************************************************/
u8_t CH9434UARTxGetRxFIFOData (u8_t uart_idx, u8_t *p_data, u16_t read_len) {
    return CH9434RegReadBytes (CH9434_UARTx_RBR_ADD + 0x10 * uart_idx, p_data, read_len);
}

/*******************************************************************************
 * Function Name  : CH9434UARTxGetTxFIFOLen
 * Description    : Obtain the length of the serial port transmission FIFO
 * Input          : uart_idx:Serial port number
 * Output         : None
 * Return         : The current length of received data on the serial port
 *******************************************************************************/
u16_t CH9434UARTxGetTxFIFOLen (u8_t uart_idx) {
    u8_t uart_fifo_ctrl = 0;
    u8_t uart_fifo_cnt_l;
    u8_t uart_fifo_cnt_h;
    u16_t uart_fifo_cnt = 0;

    uart_fifo_ctrl |= CH9434_FIFO_CTRL_TR;
    uart_fifo_ctrl |= uart_idx;
    CH9434RegWriteOneByte (CH9434_FIFO_CTRL_ADD, uart_fifo_ctrl);
    uart_fifo_cnt_l = CH9434RegReadOneByte (CH9434_FIFO_CTRL_L_ADD);
    uart_fifo_cnt_h = CH9434RegReadOneByte (CH9434_FIFO_CTRL_H_ADD);
    uart_fifo_cnt = uart_fifo_cnt_h;
    uart_fifo_cnt = (uart_fifo_cnt << 8) | uart_fifo_cnt_l;

    return uart_fifo_cnt;
}

/*******************************************************************************
* Function Name  : CH9434UARTxSetTxFIFOData
* Description    : Serial port input of transmitted data
* Input          : uart_idx:Serial port number
                   p_data:Send data pointer
                   send_len:The length of the transmitted data
* Output         : None
* Return         : None
*******************************************************************************/
u8_t CH9434UARTxSetTxFIFOData (u8_t uart_idx, u8_t *p_data, u16_t send_len) {
    return CH9434RegWriteBytes (CH9434_UARTx_THR_ADD + 0x10 * uart_idx, p_data, send_len);
}

/*******************************************************************************
* Function Name  : CH9434UARTxTnowSet
* Description    : Serial port 485 switch pin setting
* Input          : uart_idx:Serial port number
                   tnow_en:Serial port TNOW enable status
                   polar:Polarity reverse setting
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxTnowSet (u8_t uart_idx, u8_t tnow_en, u8_t polar) {
    u8_t tnow_ctrl_reg;

    tnow_ctrl_reg = CH9434RegReadOneByte (CH9434_TNOW_CTRL_CFG_ADD);
    if (tnow_en)
        tnow_ctrl_reg |= (1 << uart_idx);
    else
        tnow_ctrl_reg &= ~(1 << uart_idx);
    if (polar)
        tnow_ctrl_reg |= (1 << (uart_idx + 4));
    else
        tnow_ctrl_reg &= ~(1 << (uart_idx + 4));
    CH9434RegWriteOneByte (CH9434_TNOW_CTRL_CFG_ADD, tnow_ctrl_reg);
}

/*******************************************************************************
 * Function Name  : CH9434LowerPowerModeSet
 * Description    : CH9434 Chip low-power setting
 * Input          : mode:Low-power mode
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434LowerPowerModeSet (u8_t mode) {
    lower_power_reg = mode;
    CH9434RegWriteOneByte (CH9434_SLEEP_MOD_CFG_ADD, lower_power_reg);
}

/*******************************************************************************
 * Function Name  : CH9434WakeUp
 * Description    : CH9434 Wakeup operation: It can be awakened from the low-power 
                    mode, and it can also be awakened by operating the SPI.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434WakeUp (void) {
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434GPIOFuncSet
* Description    : GPIO Function Settings
* Input          : gpio_idx:GPIO number
                   en:Enable state
                   dir:GPIO direction
                   pu:Pull-up settings
                   pd:Pull-down settings
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434GPIOFuncSet (u8_t gpio_idx, u8_t en, u8_t dir, u8_t pu, u8_t pd) {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    u8_t gpio_func_reg;  //  GPIO_FUNC
    u8_t gpio_dir_reg;
    u8_t gpio_pu_reg;
    u8_t gpio_pd_reg;

    if (en) {

        gpio_func_reg = CH9434RegReadOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8));
        gpio_func_reg |= (1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8), gpio_func_reg);


        gpio_dir_reg = CH9434RegReadOneByte (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 8));
        if (dir)
            gpio_dir_reg |= (1 << (gpio_idx % 8));
        else
            gpio_dir_reg &= ~(1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 8), gpio_dir_reg);


        gpio_pu_reg = CH9434RegReadOneByte (CH9434_GPIO_PU_MOD_0 + (gpio_idx / 8));
        if (pu)
            gpio_pu_reg |= (1 << (gpio_idx % 8));
        else
            gpio_pu_reg &= ~(1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_PU_MOD_0 + (gpio_idx / 8), gpio_pu_reg);


        gpio_pd_reg = CH9434RegReadOneByte (CH9434_GPIO_PD_MOD_0 + (gpio_idx / 8));
        if (pd)
            gpio_pd_reg |= (1 << (gpio_idx % 8));
        else
            gpio_pd_reg &= ~(1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_PD_MOD_0 + (gpio_idx / 8), gpio_pd_reg);
    } else {
        gpio_func_reg = CH9434RegReadOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8));
        gpio_func_reg &= ~(1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8), gpio_func_reg);
    }
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
    u8_t gpio_dir_mode_reg;
    u8_t io_mode_idx;
    u8_t io_dir_idx;
    u8_t gpio_func_reg; 

    if (en) {

        gpio_func_reg = CH9434RegReadOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8));
        gpio_func_reg |= (1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8), gpio_func_reg);

        gpio_dir_mode_reg = CH9434RegReadOneByte (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 2));

        if (gpio_idx % 2) {
            gpio_dir_mode_reg &= 0x0f;
            io_mode_idx = 6;
            io_dir_idx = 4;
        } else {
            gpio_dir_mode_reg &= 0xf0;
            io_mode_idx = 2;
            io_dir_idx = 0;
        }

        if (dir == CH9434_GPIO_DIR_IN) {
            if ((pu == CH9434_GPIO_PU_DISABLE) && (pd == CH9434_GPIO_PD_DISABLE)) {
                gpio_dir_mode_reg |= (CH9434_GPIO_Mode_IN_FLOATING << io_mode_idx);
                gpio_dir_mode_reg |= (CH9434_GPIO_Mode_IN << io_dir_idx);
            } else {
                gpio_dir_mode_reg |= (CH9434_GPIO_Mode_IPD_OR_IPU << io_mode_idx);
                gpio_dir_mode_reg |= (CH9434_GPIO_Mode_IN << io_dir_idx);
                if (pu == CH9434_GPIO_PU_ENABLE) {
                    CH9434RegWriteOneByte (CH9434_GPIO_SET_0 + (gpio_idx / 8), 1 << (gpio_idx % 8));
                } else if (pd == CH9434_GPIO_PD_ENABLE) {
                    CH9434RegWriteOneByte (CH9434_GPIO_RESET_0 + (gpio_idx / 8), 1 << (gpio_idx % 8));
                }
            }
            CH9434RegWriteOneByte (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 2), gpio_dir_mode_reg);
        } else {
            gpio_dir_mode_reg |= (CH9434_GPIO_Mode_Out_PP << io_mode_idx);
            gpio_dir_mode_reg |= (CH9434_GPIO_Mode_OUT << io_dir_idx);
            CH9434RegWriteOneByte (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 2), gpio_dir_mode_reg);
        }
    } else {
        gpio_func_reg = CH9434RegReadOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8));
        gpio_func_reg &= ~(1 << (gpio_idx % 8));
        CH9434RegWriteOneByte (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8), gpio_func_reg);
    }
#endif
}

/*******************************************************************************
* Function Name  : CH9434GPIOPinOut
* Description    : Setting of GPIO output level
* Input          : gpio_idx:GPIO number
                   out_val:Output level setting
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434GPIOPinOut (u8_t gpio_idx, u8_t out_val) {
    u8_t pin_val_reg;
    u8_t reg_add;

    (void)pin_val_reg;
    (void)reg_add;
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    if (out_val)
        ch9434_gpio_x_val |= (1 << gpio_idx);
    else
        ch9434_gpio_x_val &= ~(1 << gpio_idx);

    pin_val_reg = (u8_t)(ch9434_gpio_x_val >> ((gpio_idx / 8) * 8));
    CH9434RegWriteOneByte (CH9434_GPIO_PIN_VAL_0 + (gpio_idx / 8), pin_val_reg);
#endif
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
    if (out_val)
        reg_add = CH9434_GPIO_SET_0 + (gpio_idx / 8);
    else
        reg_add = CH9434_GPIO_RESET_0 + (gpio_idx / 8);
    CH9434RegWriteOneByte (reg_add, 1 << (gpio_idx % 8));
#endif
}

/*******************************************************************************
 * Function Name  : CH9434GPIOPinVal
 * Description    : GPIO level reading
 * Input          : gpio_idx:GPIO number
 * Output         : None
 * Return         : Level status: 1: High level 0: Low level
 *******************************************************************************/
u8_t CH9434GPIOPinVal (u8_t gpio_idx) {
    u8_t pin_val_reg;

    pin_val_reg = CH9434RegReadOneByte (CH9434_GPIO_PIN_VAL_0 + (gpio_idx / 8));
    if (pin_val_reg & (1 << (gpio_idx % 8)))
        return 1;
    else
        return 0;
}

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

/*******************************************************************************
 * Function Name  : CH9434DefIOFuncEn
 * Description    : CH9434 default pin function enable
 * Input          : io_idx:Default pin number,CH9434D_DEF_U1_ADD ~ CH9434D_DEF_CTS4_ADD
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434DefIOFuncEn (u8_t io_idx) {
    u8_t io_cmd_buf[4];
    u32_t i, tim;

    io_cmd_buf[0] = CH9434D_IO_DEF_W_EN;
    io_cmd_buf[1] = io_idx;
    io_cmd_buf[2] = 1;
    io_cmd_buf[3] = CH9434D_IO_CMD_ACT;
    CH9434RegWriteBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
    tim = 0;
    while (tim++ < 10) {
        for (i = 0; i < 1000; i++) CH9434_US_DELAY();
        CH9434RegReadBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
        if (io_cmd_buf[3] == CH9434D_IO_CMD_COMP)
        {
           break; 
        }
            
    }
}

/*******************************************************************************
 * Function Name  : CH9434ReadDefIOFuncEnSta
 * Description    : Read the default pin function enable status of CH9434
 * Input          : io_idx:Default pin number,CH9434D_DEF_U1_ADD ~ CH9434D_DEF_CTS4_ADD
 * Output         : None
 * Return         : 0 or 1 indicate "off" and "on", while other values represent abnormalities.
 *******************************************************************************/
u8_t CH9434ReadDefIOFuncEnSta (u8_t io_idx) {
    u8_t io_cmd_buf[4];
    u32_t i, tim;

    io_cmd_buf[0] = CH9434D_IO_DEF_R_EN;
    io_cmd_buf[1] = io_idx;
    io_cmd_buf[2] = 0xfd;
    io_cmd_buf[3] = CH9434D_IO_CMD_ACT;

    CH9434RegWriteBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
    tim = 0;
    while (tim++ < 10) {
        for (i = 0; i < 1000; i++) CH9434_US_DELAY();
        CH9434RegReadBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
        if (io_cmd_buf[3] == CH9434D_IO_CMD_COMP)
            break;
    }
    if (tim >= 10)
        return 0xfe;
    return io_cmd_buf[2];
}

/*******************************************************************************
 * Function Name  : CH9434MulIOFuncEn
 * Description    : CH9434 Multiplexing Pin Function Enable
 * Input          : io_idx:Default pin number,CH9434D_MUL_INT_ADD ~ CH9434D_MUL_GPIO4_ADD
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434MulIOFuncEn (u8_t io_idx) {
    u8_t io_cmd_buf[4];
    u32_t i, tim;

    io_cmd_buf[0] = CH9434D_IO_MULTI_W_EN;
    io_cmd_buf[1] = io_idx;
    io_cmd_buf[2] = 1;
    io_cmd_buf[3] = CH9434D_IO_CMD_ACT;

    CH9434RegWriteBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
    tim = 0;
    while (tim++ < 10) {
        for (i = 0; i < 1000; i++) CH9434_US_DELAY();
        CH9434RegReadBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
        if (io_cmd_buf[3] == CH9434D_IO_CMD_COMP)
            break;
    }
}

/*******************************************************************************
 * Function Name  : CH9434ReadMulIOFuncEnSta
 * Description    : Read the function enable status of the CH9434 multiplexing pin
 * Input          : io_idx:Default pin number,CH9434D_MUL_INT_ADD ~ CH9434D_MUL_GPIO4_ADD
 * Output         : None
 * Return         : 0 or 1 indicate "off" and "on", while other values represent abnormalities.
 *******************************************************************************/
u8_t CH9434ReadMulIOFuncEnSta (u8_t io_idx) {
    u8_t io_cmd_buf[4];
    u32_t i, tim;

    io_cmd_buf[0] = CH9434D_IO_MULTI_R_EN;
    io_cmd_buf[1] = io_idx;
    io_cmd_buf[2] = 0xfd;
    io_cmd_buf[3] = CH9434D_IO_CMD_ACT;

    CH9434RegWriteBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
    tim = 0;
    while (tim++ < 10) {
        for (i = 0; i < 1000; i++) CH9434_US_DELAY();
        CH9434RegReadBytes (CH9434_IO_SEL_FUN_CFG, io_cmd_buf, 4);
        if (io_cmd_buf[3] == CH9434D_IO_CMD_COMP)
            break;
    }
    if (tim >= 10)
        return 0xfe;
    return io_cmd_buf[2];
}


/*******************************************************************************
 * Function Name  : CH9434WriteCANReg
 * Description    : Write the CAN registers of CH9434
 * Input          : reg_add:CAN register address
                    reg_val:The value written into the register
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434WriteCANReg (u8_t reg_add, u32_t reg_val) {
    u8_t dat[5];

    dat[0] = reg_add;
    dat[1] = reg_val & 0xff;
    dat[2] = (reg_val >> 8) & 0xff;
    dat[3] = (reg_val >> 16) & 0xff;
    dat[4] = (reg_val >> 24) & 0xff;

    CH9434RegWriteBytes (CH9434D_CAN_REG, dat, 5);
}

/*******************************************************************************
 * Function Name  : CH9434ReadCANReg
 * Description    : Read the CAN registers of CH9434
 * Input          : reg_add:CAN register address
 * Output         : None
 * Return         : Return the value of this register
 *******************************************************************************/
u32_t CH9434ReadCANReg (u8_t reg_add) {
    u32_t reg_val_t;
    u8_t dat[4];
    u8_t i;

#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_READ | CH9434D_CAN_REG);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_WRITE_BYTE (reg_add);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    for (i = 0; i < 4; i++) {
        dat[i] = CH9434_SPI_WRITE_BYTE (0xff);
        CH9434_US_DELAY();
    }
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
#endif

#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_READ | CH9434D_CAN_REG);
    CH9434_IIC_SEND_DATA (reg_add);

    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_R | ch9434d_iic_add);
    for (i = 0; i < 4; i++) dat[i] = CH9434_IIC_READ_DATA();
    CH9434_IIC_STOP();
#endif

    reg_val_t = 0;
    reg_val_t |= dat[0];
    reg_val_t |= dat[1] << 8;
    reg_val_t |= dat[2] << 16;
    reg_val_t |= dat[3] << 24;

    return reg_val_t;
}

/*******************************************************************************
 * Function Name  : CH9434D_CAN_Init
 * Description    : CAN interface initialization
 * Input          : CAN_InitStruct:CAN parameter settings
 * Output         : None
 * Return         : Initialization state
 *******************************************************************************/
u8_t CH9434D_CAN_Init (CH9434D_CAN_InitTypeDef *CAN_InitStruct) {
    u8_t InitStatus = CH9434D_CAN_InitStatus_Failed;
    u32_t wait_ack = 0x00000000;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
    reg_val &= ~CH9434D_CAN_CTLR_SLEEP;
    CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
    reg_val |= CH9434D_CAN_CTLR_INRQ;
    CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

    while (((CH9434ReadCANReg (CH9434D_CAN_STATR) & CH9434D_CAN_STATR_INAK) != CH9434D_CAN_STATR_INAK) && (wait_ack != CH9434D_INAK_TIMEOUT)) {
        wait_ack++;
    }

    if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & CH9434D_CAN_STATR_INAK) != CH9434D_CAN_STATR_INAK) {
        InitStatus = CH9434D_CAN_InitStatus_Failed;
    } else {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);

        if (CAN_InitStruct->CAN_TTCM)
            reg_val |= CH9434D_CAN_CTLR_TTCM;
        else
            reg_val &= ~(u32_t)CH9434D_CAN_CTLR_TTCM;

        if (CAN_InitStruct->CAN_ABOM)
            reg_val |= CH9434D_CAN_CTLR_ABOM;
        else
            reg_val &= ~(u32_t)CH9434D_CAN_CTLR_ABOM;

        if (CAN_InitStruct->CAN_AWUM)
            reg_val |= CH9434D_CAN_CTLR_AWUM;
        else
            reg_val &= ~(u32_t)CH9434D_CAN_CTLR_AWUM;

        if (CAN_InitStruct->CAN_NART)
            reg_val |= CH9434D_CAN_CTLR_NART;
        else
            reg_val &= ~(u32_t)CH9434D_CAN_CTLR_NART;

        if (CAN_InitStruct->CAN_RFLM)
            reg_val |= CH9434D_CAN_CTLR_RFLM;
        else
            reg_val &= ~(u32_t)CH9434D_CAN_CTLR_RFLM;

        if (CAN_InitStruct->CAN_TXFP)
            reg_val |= CH9434D_CAN_CTLR_TXFP;
        else
            reg_val &= ~(u32_t)CH9434D_CAN_CTLR_TXFP;

        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        reg_val = (u32_t)((u32_t)CAN_InitStruct->CAN_Mode << 30) |
                  ((u32_t)CAN_InitStruct->CAN_SJW << 24) |
                  ((u32_t)CAN_InitStruct->CAN_BS1 << 16) |
                  ((u32_t)CAN_InitStruct->CAN_BS2 << 20) |
                  ((u32_t)CAN_InitStruct->CAN_Prescaler - 1);

        CH9434WriteCANReg (CH9434D_CAN_BTIMR, reg_val);

        reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
        reg_val &= ~(u32_t)CH9434D_CAN_CTLR_INRQ;
        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        wait_ack = 0;

        while (((CH9434ReadCANReg (CH9434D_CAN_STATR) & CH9434D_CAN_STATR_INAK) == CH9434D_CAN_STATR_INAK) && (wait_ack != CH9434D_INAK_TIMEOUT)) {
            wait_ack++;
        }

        if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & CH9434D_CAN_STATR_INAK) == CH9434D_CAN_STATR_INAK) {
            InitStatus = CH9434D_CAN_InitStatus_Failed;
        } else {
            InitStatus = CH9434D_CAN_InitStatus_Success;
        }
    }

    return InitStatus;
}

/*******************************************************************************
 * Function Name  : CAN_FilterInit
 * Description    : CAN Filter Settings
 * Input          : CAN_FilterInitStruct:Filter settings parameters
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_FilterInit (CH9434D_CAN_FilterInitTypeDef *CAN_FilterInitStruct) {
    u32_t filter_number_bit_pos = 0;
    u32_t reg_val;

    filter_number_bit_pos = ((u32_t)1) << CAN_FilterInitStruct->CAN_FilterNumber;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_FCTLR);
    reg_val |= CH9434D_CAN_FCTLR_FINIT;
    CH9434WriteCANReg (CH9434D_CAN_FCTLR, reg_val);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_FWR);
    reg_val &= ~(u32_t)filter_number_bit_pos;
    CH9434WriteCANReg (CH9434D_CAN_FWR, reg_val);

    if (CAN_FilterInitStruct->CAN_FilterScale == CH9434D_CAN_FilterScale_16bit) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_FSCFGR);
        reg_val &= ~(u32_t)filter_number_bit_pos;
        CH9434WriteCANReg (CH9434D_CAN_FSCFGR, reg_val);

        reg_val = ((0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow) << 16) |
                  (0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterIdLow);
        CH9434WriteCANReg (CH9434D_CAN_FxR1 (CAN_FilterInitStruct->CAN_FilterNumber), reg_val);

        reg_val = ((0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
                  (0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterIdHigh);
        CH9434WriteCANReg (CH9434D_CAN_FxR2 (CAN_FilterInitStruct->CAN_FilterNumber), reg_val);
    }

    if (CAN_FilterInitStruct->CAN_FilterScale == CH9434D_CAN_FilterScale_32bit) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_FSCFGR);
        reg_val |= filter_number_bit_pos;
        CH9434WriteCANReg (CH9434D_CAN_FSCFGR, reg_val);

        reg_val = ((0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterIdHigh) << 16) |
                  (0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterIdLow);
        CH9434WriteCANReg (CH9434D_CAN_FxR1 (CAN_FilterInitStruct->CAN_FilterNumber), reg_val);

        reg_val = ((0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
                  (0x0000FFFF & (u32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow);
        CH9434WriteCANReg (CH9434D_CAN_FxR2 (CAN_FilterInitStruct->CAN_FilterNumber), reg_val);
    }

    reg_val = CH9434ReadCANReg (CH9434D_CAN_FMCFGR);
    if (CAN_FilterInitStruct->CAN_FilterMode == CH9434D_CAN_FilterMode_IdMask) {
        reg_val &= ~(u32_t)filter_number_bit_pos;
    } else {
        reg_val |= (u32_t)filter_number_bit_pos;
    }
    CH9434WriteCANReg (CH9434D_CAN_FMCFGR, reg_val);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_FAFIFOR);
    if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CH9434D_CAN_Filter_FIFO0) {
        reg_val &= ~(u32_t)filter_number_bit_pos;
    }

    if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CH9434D_CAN_Filter_FIFO1) {
        reg_val |= (u32_t)filter_number_bit_pos;
    }
    CH9434WriteCANReg (CH9434D_CAN_FAFIFOR, reg_val);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_FWR);
    if (CAN_FilterInitStruct->CAN_FilterActivation == CH9434_ENABLE) {
        reg_val |= filter_number_bit_pos;
    }
    CH9434WriteCANReg (CH9434D_CAN_FWR, reg_val);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_FCTLR);
    reg_val &= ~CH9434D_CAN_FCTLR_FINIT;
    CH9434WriteCANReg (CH9434D_CAN_FCTLR, reg_val);
}

/*******************************************************************************
 * Function Name  : CAN_StructInit
 * Description    : Initialize CAN parameter settings
 * Input          : CAN_InitStruct:CAN parameter settings
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_StructInit (CH9434D_CAN_InitTypeDef *CAN_InitStruct) {
    CAN_InitStruct->CAN_TTCM = CH9434_DISABLE;
    CAN_InitStruct->CAN_ABOM = CH9434_DISABLE;
    CAN_InitStruct->CAN_AWUM = CH9434_DISABLE;
    CAN_InitStruct->CAN_NART = CH9434_DISABLE;
    CAN_InitStruct->CAN_RFLM = CH9434_DISABLE;
    CAN_InitStruct->CAN_TXFP = CH9434_DISABLE;
    CAN_InitStruct->CAN_Mode = CH9434D_CAN_Mode_Normal;
    CAN_InitStruct->CAN_SJW = CH9434D_CAN_SJW_tq (1);
    CAN_InitStruct->CAN_BS1 = CH9434D_CAN_BS1_tq (4);
    CAN_InitStruct->CAN_BS2 = CH9434D_CAN_BS2_tq (3);
    CAN_InitStruct->CAN_Prescaler = 1;
}

/*******************************************************************************
 * Function Name  : CAN_TTComModeCmd
 * Description    : CAN Time-Triggered Mode Settings
 * Input          : NewState:Enable or disable the setting
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_TTComModeCmd (CH9434_FuncSta NewState) {
    u32_t reg_val;
    u8_t i;

    if (NewState != CH9434_DISABLE) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
        reg_val |= CH9434D_CAN_CTLR_TTCM;
        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        for (i = 0; i < 3; i++) {
            reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMDTR0 + i * 4);
            reg_val |= CH9434D_CAN_TXMDTRx_TGT;
            CH9434WriteCANReg (CH9434D_CAN_TXMDTR0 + i * 4, reg_val);
        }
    } else {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
        reg_val &= (u32_t)(~(u32_t)CH9434D_CAN_CTLR_TTCM);
        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        for (i = 0; i < 3; i++) {
            reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMDTR0 + i * 4);
            reg_val &= ((u32_t)~CH9434D_CAN_TXMDTRx_TGT);
            CH9434WriteCANReg (CH9434D_CAN_TXMDTR0 + i * 4, reg_val);
        }
    }
}

/*******************************************************************************
 * Function Name  : CAN_Transmit
 * Description    : CAN transmission
 * Input          : TxMessage:Send email parameters
 * Output         : None
 * Return         : The serial number of the sent email
 *******************************************************************************/
u8_t CH9434D_CAN_Transmit (CH9434D_CanTxMsg *TxMessage) {
    u8_t transmit_mailbox = 0;
    u32_t reg_val;

    if ((CH9434ReadCANReg (CH9434D_CAN_TSTATR) & CH9434D_CAN_TSTATR_TME0) == CH9434D_CAN_TSTATR_TME0) {
        transmit_mailbox = 0;
    } else if ((CH9434ReadCANReg (CH9434D_CAN_TSTATR) & CH9434D_CAN_TSTATR_TME1) == CH9434D_CAN_TSTATR_TME1) {
        transmit_mailbox = 1;
    } else if ((CH9434ReadCANReg (CH9434D_CAN_TSTATR) & CH9434D_CAN_TSTATR_TME2) == CH9434D_CAN_TSTATR_TME2) {
        transmit_mailbox = 2;
    } else {
        transmit_mailbox = CH9434D_CAN_TxStatus_NoMailBox;
    }

    if (transmit_mailbox != CH9434D_CAN_TxStatus_NoMailBox) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4);
        reg_val &= CH9434D_CAN_TXMIRx_TXRQ;
        CH9434WriteCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4, reg_val);

        if (TxMessage->IDE == CH9434D_CAN_Id_Standard) {
            reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4);
            reg_val |= ((TxMessage->StdId << 21) | TxMessage->RTR);
            CH9434WriteCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4, reg_val);
        } else {
            reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4);
            reg_val |= ((TxMessage->ExtId << 3) | TxMessage->IDE | TxMessage->RTR);
            CH9434WriteCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4, reg_val);
        }

        TxMessage->DLC &= (u8_t)0x0000000F;

        reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMDTR0 + transmit_mailbox * 4);
        reg_val &= (u32_t)0xFFFFFFF0;
        reg_val |= TxMessage->DLC;
        CH9434WriteCANReg (CH9434D_CAN_TXMDTR0 + transmit_mailbox * 4, reg_val);

        reg_val = (((u32_t)TxMessage->Data[3] << 24) |
                   ((u32_t)TxMessage->Data[2] << 16) |
                   ((u32_t)TxMessage->Data[1] << 8) |
                   ((u32_t)TxMessage->Data[0]));
        CH9434WriteCANReg (CH9434D_CAN_TXMDLR0 + transmit_mailbox * 4, reg_val);

        reg_val = (((u32_t)TxMessage->Data[7] << 24) |
                   ((u32_t)TxMessage->Data[6] << 16) |
                   ((u32_t)TxMessage->Data[5] << 8) |
                   ((u32_t)TxMessage->Data[4]));
        CH9434WriteCANReg (CH9434D_CAN_TXMDHR0 + transmit_mailbox * 4, reg_val);

        reg_val = CH9434ReadCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4);
        reg_val |= CH9434D_CAN_TXMIRx_TXRQ;
        CH9434WriteCANReg (CH9434D_CAN_TXMIR0 + transmit_mailbox * 4, reg_val);
    }

    return transmit_mailbox;
}

/*******************************************************************************
 * Function Name  : CAN_TransmitStatus
 * Description    : CAN Status Query Transmission
 * Input          : TransmitMailbox:Mailbox Number
 * Output         : None
 * Return         : The status of this email account
 *******************************************************************************/
u8_t CH9434D_CAN_TransmitStatus (u8_t TransmitMailbox) {
    u32_t state = 0;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_TSTATR);
    switch (TransmitMailbox) {
    case (CH9434D_CAN_Transmit_Mailbox0):
        state = reg_val & (CH9434D_CAN_TSTATR_RQCP0 | CH9434D_CAN_TSTATR_TXOK0 | CH9434D_CAN_TSTATR_TME0);
        break;

    case (CH9434D_CAN_Transmit_Mailbox1):
        state = reg_val & (CH9434D_CAN_TSTATR_RQCP1 | CH9434D_CAN_TSTATR_TXOK1 | CH9434D_CAN_TSTATR_TME1);
        break;

    case (CH9434D_CAN_Transmit_Mailbox2):
        state = reg_val & (CH9434D_CAN_TSTATR_RQCP2 | CH9434D_CAN_TSTATR_TXOK2 | CH9434D_CAN_TSTATR_TME2);
        break;

    default:
        state = CH9434D_CAN_TxStatus_Failed;
        break;
    }

    switch (state) {
    case (0x0):
        state = CH9434D_CAN_TxStatus_Pending;
        break;

    case (CH9434D_CAN_TSTATR_RQCP0 | CH9434D_CAN_TSTATR_TME0):
        state = CH9434D_CAN_TxStatus_Failed;
        break;

    case (CH9434D_CAN_TSTATR_RQCP1 | CH9434D_CAN_TSTATR_TME1):
        state = CH9434D_CAN_TxStatus_Failed;
        break;

    case (CH9434D_CAN_TSTATR_RQCP2 | CH9434D_CAN_TSTATR_TME2):
        state = CH9434D_CAN_TxStatus_Failed;
        break;

    case (CH9434D_CAN_TSTATR_RQCP0 | CH9434D_CAN_TSTATR_TXOK0 | CH9434D_CAN_TSTATR_TME0):
        state = CH9434D_CAN_TxStatus_Ok;
        break;

    case (CH9434D_CAN_TSTATR_RQCP1 | CH9434D_CAN_TSTATR_TXOK1 | CH9434D_CAN_TSTATR_TME1):
        state = CH9434D_CAN_TxStatus_Ok;
        break;

    case (CH9434D_CAN_TSTATR_RQCP2 | CH9434D_CAN_TSTATR_TXOK2 | CH9434D_CAN_TSTATR_TME2):
        state = CH9434D_CAN_TxStatus_Ok;
        break;

    default:
        state = CH9434D_CAN_TxStatus_Failed;
        break;
    }

    return (u8_t)state;
}

/*******************************************************************************
 * Function Name  : CAN_CancelTransmit
 * Description    : CAN has cancelled the transmission.
 * Input          : Mailbox:Mailbox Number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_CancelTransmit (u8_t Mailbox) {
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_TSTATR);
    switch (Mailbox) {
    case (CH9434D_CAN_Transmit_Mailbox0): {
        reg_val |= CH9434D_CAN_TSTATR_ABRQ0;
        CH9434WriteCANReg (CH9434D_CAN_TSTATR, reg_val);
        break;
    }
    case (CH9434D_CAN_Transmit_Mailbox1): {
        reg_val |= CH9434D_CAN_TSTATR_ABRQ1;
        CH9434WriteCANReg (CH9434D_CAN_TSTATR, reg_val);
        break;
    }
    case (CH9434D_CAN_Transmit_Mailbox2): {
        reg_val |= CH9434D_CAN_TSTATR_ABRQ2;
        CH9434WriteCANReg (CH9434D_CAN_TSTATR, reg_val);
        break;
    }

    default:
        break;
    }
}

/*******************************************************************************
 * Function Name  : CAN_Receive
 * Description    : CAN receives and then releases this message upon completion.
 * Input          : FIFONumber:FIFO sequence number
                    RxMessage:Email parameter address
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_Receive (u8_t FIFONumber, CH9434D_CanRxMsg *RxMessage) {
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_RXMIR0 + FIFONumber * 4);
    RxMessage->IDE = (u8_t)0x04 & reg_val;

    if (RxMessage->IDE == CH9434D_CAN_Id_Standard) {
        RxMessage->StdId = (u32_t)0x000007FF & (reg_val >> 21);
    } else {
        RxMessage->ExtId = (u32_t)0x1FFFFFFF & (reg_val >> 3);
    }

    RxMessage->RTR = (u8_t)0x02 & reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_RXMDTR0 + FIFONumber * 4);
    RxMessage->DLC = (u8_t)0x0F & reg_val;
    RxMessage->FMI = (u8_t)0xFF & (reg_val >> 8);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_RXMDLR0 + FIFONumber * 4);
    RxMessage->Data[0] = (u8_t)0xFF & reg_val;
    RxMessage->Data[1] = (u8_t)0xFF & (reg_val >> 8);
    RxMessage->Data[2] = (u8_t)0xFF & (reg_val >> 16);
    RxMessage->Data[3] = (u8_t)0xFF & (reg_val >> 24);

    reg_val = CH9434ReadCANReg (CH9434D_CAN_RXMDHR0 + FIFONumber * 4);
    RxMessage->Data[4] = (u8_t)0xFF & reg_val;
    RxMessage->Data[5] = (u8_t)0xFF & (reg_val >> 8);
    RxMessage->Data[6] = (u8_t)0xFF & (reg_val >> 16);
    RxMessage->Data[7] = (u8_t)0xFF & (reg_val >> 24);


    reg_val = CH9434ReadCANReg (CH9434D_CAN_RFIFO0 + FIFONumber);
    reg_val |= CH9434D_CAN_RFIFOx_RFOMx;
    CH9434WriteCANReg (CH9434D_CAN_RFIFO0 + FIFONumber, reg_val);
}

/*******************************************************************************
 * Function Name  : CAN_FIFORelease
 * Description    : CAN release the current mailbox message
 * Input          : FIFONumber:FIFO sequence number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_FIFORelease (u8_t FIFONumber) {
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_RFIFO0 + FIFONumber);
    reg_val |= CH9434D_CAN_RFIFOx_RFOMx;
    CH9434WriteCANReg (CH9434D_CAN_RFIFO0 + FIFONumber, reg_val);
}

/*******************************************************************************
 * Function Name  : CAN_MessagePending
 * Description    : CAN query the number of messages in the FIFO buffer
 * Input          : FIFONumber:FIFO sequence number
 * Output         : None
 * Return         : None
 *******************************************************************************/
u8_t CH9434D_CAN_MessagePending (u8_t FIFONumber) {
    return (u8_t)(CH9434ReadCANReg (CH9434D_CAN_RFIFO0 + FIFONumber) & (u32_t)CH9434D_CAN_RFIFOx_FMPx);
}

/*******************************************************************************
 * Function Name  : CAN_OperatingModeRequest
 * Description    : CAN Current Mode Selection
 * Input          : CAN_OperatingMode:CAN mode
 * Output         : None
 * Return         : Execution status
 *******************************************************************************/
u8_t CH9434D_CAN_OperatingModeRequest (u8_t CAN_OperatingMode) {
    u8_t status = CH9434D_CAN_ModeStatus_Failed;
    u32_t timeout = CH9434D_INAK_TIMEOUT;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
    if (CAN_OperatingMode == CH9434D_CAN_OperatingMode_Initialization) {
        reg_val = (u32_t)((reg_val & (u32_t)(~(u32_t)CH9434D_CAN_CTLR_SLEEP)) | CH9434D_CAN_CTLR_INRQ);
        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        while (((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) != CH9434D_CAN_STATR_INAK) && (timeout != 0)) {
            timeout--;
        }
        if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) != CH9434D_CAN_STATR_INAK) {
            status = CH9434D_CAN_ModeStatus_Failed;
        } else {
            status = CH9434D_CAN_ModeStatus_Success;
        }
    } else if (CAN_OperatingMode == CH9434D_CAN_OperatingMode_Normal) {
        reg_val &= (u32_t)(~(CH9434D_CAN_CTLR_SLEEP | CH9434D_CAN_CTLR_INRQ));
        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        while (((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) != 0) && (timeout != 0)) {
            timeout--;
        }
        if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) != 0) {
            status = CH9434D_CAN_ModeStatus_Failed;
        } else {
            status = CH9434D_CAN_ModeStatus_Success;
        }
    } else if (CAN_OperatingMode == CH9434D_CAN_OperatingMode_Sleep) {
        reg_val = (u32_t)((reg_val & (u32_t)(~(u32_t)CH9434D_CAN_CTLR_INRQ)) | CH9434D_CAN_CTLR_SLEEP);
        CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

        while (((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) != CH9434D_CAN_STATR_SLAK) && (timeout != 0)) {
            timeout--;
        }
        if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) != CH9434D_CAN_STATR_SLAK) {
            status = CH9434D_CAN_ModeStatus_Failed;
        } else {
            status = CH9434D_CAN_ModeStatus_Success;
        }
    } else {
        status = CH9434D_CAN_ModeStatus_Failed;
    }

    return (u8_t)status;
}

/*******************************************************************************
 * Function Name  : CAN_Sleep
 * Description    : CAN enters sleep mode
 * Input          : None
 * Output         : None
 * Return         : Execution status
 *******************************************************************************/
u8_t CH9434D_CAN_Sleep (void) {
    u8_t sleepstatus = CH9434D_CAN_Sleep_Failed;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
    reg_val = (((reg_val) & (u32_t)(~(u32_t)CH9434D_CAN_CTLR_INRQ)) | CH9434D_CAN_CTLR_SLEEP);
    CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

    if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & (CH9434D_CAN_STATR_SLAK | CH9434D_CAN_STATR_INAK)) == CH9434D_CAN_STATR_SLAK) {
        sleepstatus = CH9434D_CAN_Sleep_Ok;
    }

    return (u8_t)sleepstatus;
}

/*******************************************************************************
 * Function Name  : CAN_WakeUp
 * Description    : CAN exits sleep mode
 * Input          : None
 * Output         : None
 * Return         : Execution status
 *******************************************************************************/
u8_t CH9434D_CAN_WakeUp (void) {
    u32_t wait_slak = CH9434D_SLAK_TIMEOUT;
    u8_t wakeupstatus = CH9434D_CAN_WakeUp_Failed;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_CTLR);
    reg_val &= ~(u32_t)CH9434D_CAN_CTLR_SLEEP;
    CH9434WriteCANReg (CH9434D_CAN_CTLR, reg_val);

    while (((CH9434ReadCANReg (CH9434D_CAN_STATR) & CH9434D_CAN_STATR_SLAK) == CH9434D_CAN_STATR_SLAK) && (wait_slak != 0x00)) {
        wait_slak--;
    }
    if ((CH9434ReadCANReg (CH9434D_CAN_STATR) & CH9434D_CAN_STATR_SLAK) != CH9434D_CAN_STATR_SLAK) {
        wakeupstatus = CH9434D_CAN_WakeUp_Ok;
    }

    return (u8_t)wakeupstatus;
}

/*******************************************************************************
 * Function Name  : CAN_GetLastErrorCode
 * Description    : Read the CAN LEC error code
 * Input          : None
 * Output         : None
 * Return         : LEC error code
 *******************************************************************************/
u8_t CH9434D_CAN_GetLastErrorCode (void) {
    u8_t errorcode = 0;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_ERRSR);
    errorcode = (((u8_t)reg_val) & (u8_t)CH9434D_CAN_ERRSR_LEC);

    return errorcode;
}

/*******************************************************************************
 * Function Name  : CAN_GetReceiveErrorCounter
 * Description    : Read the CAN receive error count
 * Input          : None
 * Output         : None
 * Return         : Error reception count
 *******************************************************************************/
u8_t CH9434D_CAN_GetReceiveErrorCounter (void) {
    u8_t counter = 0;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_ERRSR);
    counter = (u8_t)((reg_val & CH9434D_CAN_ERRSR_REC) >> 24);

    return counter;
}

/*******************************************************************************
 * Function Name  : CAN_GetLSBTransmitErrorCounter
 * Description    : Read the CAN transmission error count
 * Input          : None
 * Output         : None
 * Return         : Error count sent
 *******************************************************************************/
u8_t CH9434D_CAN_GetLSBTransmitErrorCounter (void) {
    u8_t counter = 0;
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_ERRSR);
    counter = (u8_t)((reg_val & CH9434D_CAN_ERRSR_TEC) >> 16);

    return counter;
}

/*******************************************************************************
 * Function Name  : CAN_ITConfig
 * Description    : CAN Interrupt Enable Configuration
 * Input          : CAN_IT:Interrupt flag
                    NewState:Enable or Disable
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_ITConfig (u32_t CAN_IT, CH9434_FuncSta NewState) {
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_INTENR);
    if (NewState != CH9434_DISABLE)
        reg_val |= CAN_IT;
    else
        reg_val &= ~CAN_IT;
    CH9434WriteCANReg (CH9434D_CAN_INTENR, reg_val);
}

/*******************************************************************************
 * Function Name  : CAN_GetFlagStatus
 * Description    : Read the status of the flag
 * Input          : CAN_FLAG:The required symbol/identifier
 *            CH9434D_CAN_FLAG_EWG.
 *            CH9434D_CAN_FLAG_EPV.
 *            CH9434D_CAN_FLAG_BOF.
 *            CH9434D_CAN_FLAG_RQCP0.
 *            CH9434D_CAN_FLAG_RQCP1.
 *            CH9434D_CAN_FLAG_RQCP2.
 *            CH9434D_CAN_FLAG_FMP1.
 *            CH9434D_CAN_FLAG_FF1.
 *            CH9434D_CAN_FLAG_FOV1.
 *            CH9434D_CAN_FLAG_FMP0.
 *            CH9434D_CAN_FLAG_FF0.
 *            CH9434D_CAN_FLAG_FOV0.
 *            CH9434D_CAN_FLAG_WKU.
 *            CH9434D_CAN_FLAG_SLAK.
 *            CH9434D_CAN_FLAG_LEC.
 * Output         : None
 * Return         : The status of this logo
 *******************************************************************************/
CH9434_FSta CH9434D_CAN_GetFlagStatus (u32_t CAN_FLAG) {
    CH9434_FSta bitstatus = RESET_T;
    u32_t reg_val = 0;

    if ((CAN_FLAG & CH9434D_CAN_FLAGS_ERRSR) != (u32_t)RESET_T) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_ERRSR);
    } else if ((CAN_FLAG & CH9434D_CAN_FLAGS_STATR) != (u32_t)RESET_T) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_STATR);
    } else if ((CAN_FLAG & CH9434D_CAN_FLAGS_TSTATR) != (u32_t)RESET_T) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_TSTATR);
    } else if ((CAN_FLAG & CH9434D_CAN_FLAGS_RFIFO0) != (u32_t)RESET_T) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_RFIFO0);
    } else if ((CAN_FLAG & CH9434D_CAN_FLAGS_RFIFO1) != (u32_t)RESET_T) {
        reg_val = CH9434ReadCANReg (CH9434D_CAN_RFIFO1);
    }

    if ((u32_t)(reg_val & (CAN_FLAG & 0x000FFFFF)) != (u32_t)RESET_T) {
        bitstatus = SET_T;
    } else {
        bitstatus = RESET_T;
    }

    return bitstatus;
}

/*******************************************************************************
 * Function Name  : CAN_ClearFlag
 * Description    : Clear CAN flag
 * Input          : CAN_FLAG:The interrupt flags that need to be cleared
 *            CH9434D_CAN_FLAG_RQCP0.
 *            CH9434D_CAN_FLAG_RQCP1.
 *            CH9434D_CAN_FLAG_RQCP2.
 *            CH9434D_CAN_FLAG_FF1.
 *            CH9434D_CAN_FLAG_FOV1.
 *            CH9434D_CAN_FLAG_FF0.
 *            CH9434D_CAN_FLAG_FOV0.
 *            CH9434D_CAN_FLAG_WKU.
 *            CH9434D_CAN_FLAG_SLAK.
 *            CH9434D_CAN_FLAG_LEC.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_ClearFlag (u32_t CAN_FLAG) {
    u32_t flagtmp = 0;

    if (CAN_FLAG == CH9434D_CAN_FLAG_LEC) {
        CH9434WriteCANReg (CH9434D_CAN_ERRSR, (u32_t)RESET_T);
    } else {
        flagtmp = CAN_FLAG & 0x000FFFFF;

        if ((CAN_FLAG & CH9434D_CAN_FLAGS_RFIFO0) != (u32_t)RESET_T) {
            CH9434WriteCANReg (CH9434D_CAN_RFIFO0, (u32_t)(flagtmp));
        } else if ((CAN_FLAG & CH9434D_CAN_FLAGS_RFIFO1) != (u32_t)RESET_T) {
            CH9434WriteCANReg (CH9434D_CAN_RFIFO1, (u32_t)(flagtmp));
        } else if ((CAN_FLAG & CH9434D_CAN_FLAGS_TSTATR) != (u32_t)RESET_T) {
            CH9434WriteCANReg (CH9434D_CAN_TSTATR, (u32_t)(flagtmp));
        } else {
            CH9434WriteCANReg (CH9434D_CAN_STATR, (u32_t)(flagtmp));
        }
    }
}

/*******************************************************************************
 * Function Name  : CAN_GetITStatus
 * Description    : CAN reads the interrupt flag
 * Input          : CAN_IT: Interrupt flags to be read
 *            CH9434D_CAN_IT_TME.
 *            CH9434D_CAN_IT_FMP0.
 *            CH9434D_CAN_IT_FF0.
 *            CH9434D_CAN_IT_FOV0.
 *            CH9434D_CAN_IT_FMP1.
 *            CH9434D_CAN_IT_FF1.
 *            CH9434D_CAN_IT_FOV1.
 *            CH9434D_CAN_IT_WKU.
 *            CH9434D_CAN_IT_SLK.
 *            CH9434D_CAN_IT_EWG.
 *            CH9434D_CAN_IT_EPV.
 *            CH9434D_CAN_IT_BOF.
 *            CH9434D_CAN_IT_LEC.
 *            CH9434D_CAN_IT_ERR.
 * Output         : None
 * Return         : Interrupt flag status
 *******************************************************************************/
CH9434_ISta CH9434D_CAN_GetITStatus (u32_t CAN_IT) {
    CH9434_ISta ISta = RESET_T;

    if ((CH9434ReadCANReg (CH9434D_CAN_INTENR) & CAN_IT) != RESET_T) {
        switch (CAN_IT) {
        case CH9434D_CAN_IT_TME:
            ISta = CheckITStatus (CH9434D_CAN_TSTATR, CH9434D_CAN_TSTATR_RQCP0 | CH9434D_CAN_TSTATR_RQCP1 | CH9434D_CAN_TSTATR_RQCP2);
            break;

        case CH9434D_CAN_IT_FMP0:
            ISta = CheckITStatus (CH9434D_CAN_RFIFO0, CH9434D_CAN_RFIFOx_FMPx);
            break;

        case CH9434D_CAN_IT_FF0:
            ISta = CheckITStatus (CH9434D_CAN_RFIFO0, CH9434D_CAN_RFIFOx_FULLx);
            break;

        case CH9434D_CAN_IT_FOV0:
            ISta = CheckITStatus (CH9434D_CAN_RFIFO0, CH9434D_CAN_RFIFOx_FOVRx);
            break;

        case CH9434D_CAN_IT_FMP1:
            ISta = CheckITStatus (CH9434D_CAN_RFIFO1, CH9434D_CAN_RFIFOx_FMPx);
            break;

        case CH9434D_CAN_IT_FF1:
            ISta = CheckITStatus (CH9434D_CAN_RFIFO1, CH9434D_CAN_RFIFOx_FULLx);
            break;

        case CH9434D_CAN_IT_FOV1:
            ISta = CheckITStatus (CH9434D_CAN_RFIFO1, CH9434D_CAN_RFIFOx_FOVRx);
            break;

        case CH9434D_CAN_IT_WKU:
            ISta = CheckITStatus (CH9434D_CAN_STATR, CH9434D_CAN_STATR_WKUI);
            break;

        case CH9434D_CAN_IT_SLK:
            ISta = CheckITStatus (CH9434D_CAN_STATR, CH9434D_CAN_STATR_SLAKI);
            break;

        case CH9434D_CAN_IT_EWG:
            ISta = CheckITStatus (CH9434D_CAN_ERRSR, CH9434D_CAN_ERRSR_EWGF);
            break;

        case CH9434D_CAN_IT_EPV:
            ISta = CheckITStatus (CH9434D_CAN_ERRSR, CH9434D_CAN_ERRSR_EPVF);
            break;

        case CH9434D_CAN_IT_BOF:
            ISta = CheckITStatus (CH9434D_CAN_ERRSR, CH9434D_CAN_ERRSR_BOFF);
            break;

        case CH9434D_CAN_IT_LEC:
            ISta = CheckITStatus (CH9434D_CAN_ERRSR, CH9434D_CAN_ERRSR_LEC);
            break;

        case CH9434D_CAN_IT_ERR:
            ISta = CheckITStatus (CH9434D_CAN_STATR, CH9434D_CAN_STATR_ERRI);
            break;

        default:
            ISta = RESET_T;
            break;
        }
    } else {
        ISta = RESET_T;
    }

    return ISta;
}

/*******************************************************************************
 * Function Name  : CAN_ClearITPendingBit
 * Description    : CAN clears the interrupt flag
 * Input          : CAN_IT:The interrupt flags that need to be cleared
 *            CH9434D_CAN_IT_TME.
 *            CH9434D_CAN_IT_FF0.
 *            CH9434D_CAN_IT_FOV0.
 *            CH9434D_CAN_IT_FF1.
 *            CH9434D_CAN_IT_FOV1.
 *            CH9434D_CAN_IT_WKU.
 *            CH9434D_CAN_IT_SLK.
 *            CH9434D_CAN_IT_EWG.
 *            CH9434D_CAN_IT_EPV.
 *            CH9434D_CAN_IT_BOF.
 *            CH9434D_CAN_IT_LEC.
 *            CH9434D_CAN_IT_ERR.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_ClearITPendingBit (u32_t CAN_IT) {
    switch (CAN_IT) {
    case CH9434D_CAN_IT_TME:
        CH9434WriteCANReg (CH9434D_CAN_TSTATR, CH9434D_CAN_TSTATR_RQCP0 | CH9434D_CAN_TSTATR_RQCP1 | CH9434D_CAN_TSTATR_RQCP2);
        break;

    case CH9434D_CAN_IT_FF0:
        CH9434WriteCANReg (CH9434D_CAN_RFIFO0, CH9434D_CAN_RFIFOx_FULLx);
        break;

    case CH9434D_CAN_IT_FOV0:
        CH9434WriteCANReg (CH9434D_CAN_RFIFO0, CH9434D_CAN_RFIFOx_FOVRx);
        break;

    case CH9434D_CAN_IT_FF1:
        CH9434WriteCANReg (CH9434D_CAN_RFIFO1, CH9434D_CAN_RFIFOx_FULLx);
        break;

    case CH9434D_CAN_IT_FOV1:
        CH9434WriteCANReg (CH9434D_CAN_RFIFO1, CH9434D_CAN_RFIFOx_FOVRx);
        break;

    case CH9434D_CAN_IT_WKU:
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_WKUI);
        break;

    case CH9434D_CAN_IT_SLK:
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_SLAKI);
        break;

    case CH9434D_CAN_IT_EWG:
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_ERRI);
        break;

    case CH9434D_CAN_IT_EPV:
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_ERRI);
        break;

    case CH9434D_CAN_IT_BOF:
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_ERRI);
        break;

    case CH9434D_CAN_IT_LEC:
        CH9434WriteCANReg (CH9434D_CAN_ERRSR, RESET_T);
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_ERRI);
        break;

    case CH9434D_CAN_IT_ERR:
        CH9434WriteCANReg (CH9434D_CAN_ERRSR, RESET_T);
        CH9434WriteCANReg (CH9434D_CAN_STATR, CH9434D_CAN_STATR_ERRI);
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * Function Name  : CheckITStatus
 * Description    : Obtain the flag status of the corresponding register
 * Input          : CAN_Reg:Register address
                    It_Bit:Corresponding position
 * Output         : None
 * Return         : Status
 *******************************************************************************/
CH9434_ISta CheckITStatus (u32_t CAN_Reg, u32_t It_Bit) {
    CH9434_ISta pendingbitstatus = RESET_T;

    if ((CH9434ReadCANReg (CAN_Reg) & It_Bit) != (u32_t)RESET_T) {
        pendingbitstatus = SET_T;
    } else {
        pendingbitstatus = RESET_T;
    }

    return pendingbitstatus;
}

/*******************************************************************************
 * Function Name  : CAN_BS1_ModeConfig
 * Description    : Set TS1 parameters in CAN
 * Input          : CAN_BS1_Mode:Mode
                    CH9434D_CAN_BS1_tq:Parameters
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_BS1_ModeConfig (u32_t CAN_BS1_Mode, u8_t CH9434D_CAN_BS1_tq) {
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_BTIMR);
    reg_val &= ~(0x000FF000);

    if (CAN_BS1_Mode == CH9434D_CAN_BS1_6bit) {
        reg_val |= (CH9434D_CAN_BS1_tq << 16);
    } else if (CAN_BS1_Mode == CH9434D_CAN_BS1_4bit) {
        reg_val |= (CH9434D_CAN_BS1_tq << 12);
    }
    CH9434WriteCANReg (CH9434D_CAN_BTIMR, reg_val);
}

/*******************************************************************************
 * Function Name  : CAN_BusOff_ErrCntConfig
 * Description    : CAN sets the offline recovery error count value
 * Input          : BusOff_ErrCnt:Reset the value
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH9434D_CAN_BusOff_ErrCntConfig (u8_t BusOff_ErrCnt) {
    u32_t reg_val;

    reg_val = CH9434ReadCANReg (CH9434D_CAN_TERR_CNT);
    reg_val &= ~(0x000000FF);
    reg_val |= (u32_t)BusOff_ErrCnt;
    CH9434WriteCANReg (CH9434D_CAN_TERR_CNT, reg_val);
}

/*******************************************************************************
 * Function Name  : CAN_TransmitDirect
 * Description    : CAN can be sent directly, using the CAN continuous operation command
 * Input          : mailbox:Email address
                    TxMessage:Send email parameters
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CAN_TransmitDirect (u8_t mailbox, CH9434D_CanTxMsg *TxMessage) {
    u32_t can_tx_mail_reg[4];  
    u32_t i;
    u8_t *p_buf;

    if (mailbox > 2)
        return;

    p_buf = (u8_t *)can_tx_mail_reg;

    TxMessage->DLC &= (u8_t)0x0000000F;
    can_tx_mail_reg[2] = 0;
    can_tx_mail_reg[2] |= TxMessage->DLC;
    can_tx_mail_reg[1] = (((u32_t)TxMessage->Data[3] << 24) |
                          ((u32_t)TxMessage->Data[2] << 16) |
                          ((u32_t)TxMessage->Data[1] << 8) |
                          ((u32_t)TxMessage->Data[0]));

    can_tx_mail_reg[0] = (((u32_t)TxMessage->Data[7] << 24) |
                          ((u32_t)TxMessage->Data[6] << 16) |
                          ((u32_t)TxMessage->Data[5] << 8) |
                          ((u32_t)TxMessage->Data[4]));
    can_tx_mail_reg[3] = 0;
    if (TxMessage->IDE == CH9434D_CAN_Id_Standard) {
        can_tx_mail_reg[3] |= ((TxMessage->StdId << 21) | TxMessage->RTR);
    } else {
        can_tx_mail_reg[3] |= ((TxMessage->ExtId << 3) | TxMessage->IDE | TxMessage->RTR);
    }
    can_tx_mail_reg[3] |= CH9434D_CAN_TXMIRx_TXRQ;

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_WRITE | CH9434D_CAN_REG);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_WRITE_BYTE (CH9434D_CAN_TX0WRITE + mailbox);
    CH9434_US_DELAY();
    for (i = 0; i < 16; i++) CH9434_SPI_WRITE_BYTE (p_buf[i]);
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
#endif
#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_WRITE | CH9434D_CAN_REG);
    CH9434_SPI_WRITE_BYTE (CH9434D_CAN_TX0WRITE + mailbox);
    for (i = 0; i < 16; i++) CH9434_IIC_SEND_DATA (p_buf[i]);
    CH9434_IIC_STOP();
#endif
#endif
}

/*******************************************************************************
 * Function Name  : CAN_Receive
 * Description    : CAN receives and then releases this message upon completion.
 * Input          : FIFONumber:FIFO sequence number
                    RxMessage:The email parameter address needs to be provided for 4 queues.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CAN_ReceiveDirect (u8_t FIFONumber, CH9434D_CanRxMsg *RxMessage) {
    u32_t reg_val;
    u32_t can_rx_mail_reg[16];
    u32_t i;
    u8_t *p_buf;

    if (FIFONumber > 1)
        return;
    p_buf = (u8_t *)can_rx_mail_reg;

#if (CH9434D_IF_SEL == CH9434D_IF_SPI)
    CH9434_SPI_SCS_OP (CH9434_DISABLE);
    CH9434_SPI_WRITE_BYTE (CH9434_REG_OP_READ | CH9434D_CAN_REG);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_WRITE_BYTE (CH9434D_CAN_RX0READ + FIFONumber);
    for (i = 0; i < 8; i++) CH9434_US_DELAY();
    for (i = 0; i < 64; i++) p_buf[i] = CH9434_SPI_WRITE_BYTE (0xff);
    CH9434_SPI_SCS_OP (CH9434_ENABLE);
    CH9434_US_DELAY();
#endif

#if (CH9434D_IF_SEL == CH9434D_IF_IIC)
    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_W | ch9434d_iic_add);
    CH9434_IIC_SEND_DATA (CH9434_REG_OP_READ | CH9434D_CAN_REG);
    CH9434_IIC_SEND_DATA (CH9434D_CAN_RX0READ + FIFONumber);

    CH9434_IIC_START();
    CH9434_IIC_SEND_ADD (CH9434D_IIC_DIR_R | ch9434d_iic_add);
    for (i = 0; i < 64; i++) p_buf[i] = CH9434_IIC_READ_DATA();
    CH9434_IIC_STOP();
#endif

    for (i = 0; i < 4; i++) {
        reg_val = can_rx_mail_reg[3 + i * 4];  // CAN_RXMIRx
        RxMessage[i].IDE = (u8_t)0x04 & reg_val;

        if (RxMessage[i].IDE == CH9434D_CAN_Id_Standard) {
            RxMessage[i].StdId = (u32_t)0x000007FF & (reg_val >> 21);
        } else {
            RxMessage[i].ExtId = (u32_t)0x1FFFFFFF & (reg_val >> 3);
        }

        RxMessage[i].RTR = (u8_t)0x02 & reg_val;

        reg_val = can_rx_mail_reg[2 + i * 4];  // CAN_RXMDTRx
        RxMessage[i].DLC = (u8_t)0x0F & reg_val;
        RxMessage[i].FMI = (u8_t)0xFF & (reg_val >> 8);

        reg_val = can_rx_mail_reg[1 + i * 4];  // CAN_RXMDLRx
        RxMessage[i].Data[0] = (u8_t)0xFF & reg_val;
        RxMessage[i].Data[1] = (u8_t)0xFF & (reg_val >> 8);
        RxMessage[i].Data[2] = (u8_t)0xFF & (reg_val >> 16);
        RxMessage[i].Data[3] = (u8_t)0xFF & (reg_val >> 24);

        reg_val = can_rx_mail_reg[0 + i * 4];  // CAN_RXMDHRx
        RxMessage[i].Data[4] = (u8_t)0xFF & reg_val;
        RxMessage[i].Data[5] = (u8_t)0xFF & (reg_val >> 8);
        RxMessage[i].Data[6] = (u8_t)0xFF & (reg_val >> 16);
        RxMessage[i].Data[7] = (u8_t)0xFF & (reg_val >> 24);
    }
}


#endif
