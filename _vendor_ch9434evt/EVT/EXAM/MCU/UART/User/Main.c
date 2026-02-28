/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/08/08
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "SPI_Init.h"
#include "IIC_Init.h"
#include "CH9434.H"

static u8_t uart_idx = 0;
static u8_t uart_iir = 0;
static u8_t uart_lsr = 0;
static u8_t uart_msr = 0;
static u16_t rec_buf_cnt = 0;
u8_t uart_rec_buf[1024 * 10] = {0};  // Receiving buffer zone
uint32_t Version = 0;

typedef struct {

    u32_t bps;
    u8_t idx;         // Serial port number
    u8_t data_bits;   // Data bit
    u8_t stop_bits;   // Stop bit
    u8_t veri_bits;   // Check bit
    u8_t fifo_en;     // FIFO Enable
    u8_t fifo_level;  // Trigger point setting
    u8_t flow_en;     // Flow control enabled
    u8_t modem;       // Modem signal interruption
    u8_t line;        // Line status interrupted
    u8_t tx;          // Send interruption
    u8_t rx;          // Receiving interruption
    u8_t rts_val;     // The level state of the RTS pin
    u8_t dtr_val;     // DTR pin level status

} CH9434_UART_CFG;

CH9434_UART_CFG uart_cfg;

void CH9434_UARTX_INT (CH9434_UART_CFG *cfg) {

    CH9434UARTxParaSet (cfg->idx,
                        cfg->bps,
                        cfg->data_bits,
                        cfg->stop_bits,
                        cfg->veri_bits);

    CH9434UARTxFIFOSet (cfg->idx,
                        cfg->fifo_en,
                        cfg->fifo_level);

    CH9434UARTxFlowSet (cfg->idx,
                        cfg->flow_en);

    CH9434UARTxIrqSet (cfg->idx,
                       cfg->modem,
                       cfg->line,
                       cfg->tx,
                       cfg->rx);

    CH9434UARTxIrqOpen (cfg->idx);

    CH9434UARTxRtsDtrPin (cfg->idx,
                          cfg->rts_val,
                          cfg->dtr_val);
}

void CH9434_UART_FIFO_SEND (u8_t uart_idx, u8_t *data, u32_t len) {
    u32_t send_fifo_len = 0;
    while (len > 0) {
        send_fifo_len = CH9434UARTxGetTxFIFOLen (uart_idx);
        if (len <= send_fifo_len) {
            CH9434UARTxSetTxFIFOData (uart_idx, data, len);
            len = 0;
        } else {
            CH9434UARTxSetTxFIFOData (uart_idx, data, send_fifo_len);
            len -= send_fifo_len;
        }
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main (void) {

    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init (115200);
    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("This is the UART test program for CH9434.\r\n");

#if ((USE_CHIP_MODEL == CHIP_MODEL_CH9434D) && (CH9434D_IF_SEL == CH9434D_IF_IIC))
    CH9434_IIC_INIT();
#else
    CH9434_SPI_Init();
#endif

    CH9434_INT_Init();
    Delay_Ms (500);  // During synchronous power-on, wait for the CH9434 to complete its startup.

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    CH9434InitClkMode (CH9434_ENABLE, CH9434_ENABLE, 13);
    Delay_Ms (10);
#elif (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
    CH9434OscXtFreqSet (16000000);  // Record the frequency of the external clock.
    CH9434DefIOFuncEn (5);          // Enable external crystal oscillator.
    CH9434InitClkMode (CH9434_ENABLE, CH9434_ENABLE, 1);
    Delay_Ms (10);

    CH9434DefIOFuncEn (1);  //  UART1 ENABLE
    CH9434DefIOFuncEn (2);  //  UART2 ENABLE
    CH9434DefIOFuncEn (3);  //  UART3 ENABLE
    CH9434DefIOFuncEn (4);  //  UART4 ENABLE

    CH9434MulIOFuncEn (1);  //  Interruption enable.
#endif

    uart_cfg.bps = 115200;
    uart_cfg.idx = CH9434_UART_IDX_0;
    uart_cfg.data_bits = CH9434_UART_8_BITS_PER_CHAR;
    uart_cfg.stop_bits = CH9434_UART_ONE_STOP_BIT;
    uart_cfg.veri_bits = CH9434_UART_NO_PARITY;
    uart_cfg.fifo_en = CH9434_ENABLE;
    uart_cfg.fifo_level = CH9434_UART_FIFO_MODE_1280;
    uart_cfg.flow_en = CH9434_DISABLE;
    uart_cfg.modem = CH9434_DISABLE;
    uart_cfg.line = CH9434_DISABLE;
    uart_cfg.tx = CH9434_ENABLE;
    uart_cfg.rx = CH9434_ENABLE;
    uart_cfg.rts_val = CH9434_DISABLE;
    uart_cfg.dtr_val = CH9434_DISABLE;
    CH9434_UARTX_INT (&uart_cfg);

    uart_cfg.idx = CH9434_UART_IDX_1;
    CH9434_UARTX_INT (&uart_cfg);

    uart_cfg.idx = CH9434_UART_IDX_2;
    CH9434_UARTX_INT (&uart_cfg);

    uart_cfg.idx = CH9434_UART_IDX_3;
    CH9434_UARTX_INT (&uart_cfg);

    while (1) {

        /* Serial port thread processing */
        if (GPIO_ReadInputDataBit (GPIOB, GPIO_Pin_9) == Bit_RESET)  // The INT pin is effective when it is at a low level.
        {
            for (uart_idx = 0; uart_idx < 4; uart_idx++) {
                uart_iir = CH9434UARTxReadIIR (uart_idx);  // Interruption Query
                printf ("uart[%d]\r\n", uart_idx);
                printf ("iir:%02x\r\n", uart_iir);

                switch (uart_iir & 0x0f) {
                case 0x01:  // No interruption

                    break;
                case 0x06:  // Received line status
                {
                    uart_lsr = CH9434UARTxReadLSR (uart_idx);
                    printf ("lsr:%02x\r\n", uart_lsr);
                    rec_buf_cnt = CH9434UARTxGetRxFIFOLen (uart_idx);
                    printf ("len :%d\r\n", rec_buf_cnt);

                    if (rec_buf_cnt) {
                        CH9434UARTxGetRxFIFOData (uart_idx, uart_rec_buf, rec_buf_cnt);
                        CH9434_UART_FIFO_SEND (uart_idx, uart_rec_buf, rec_buf_cnt);
                    }
                    break;
                }

                case 0x04:  // Data reception is available
                {
                    uart_lsr = CH9434UARTxReadLSR (uart_idx);
                    printf ("lsr:%02x\r\n", uart_lsr);
                    rec_buf_cnt = CH9434UARTxGetRxFIFOLen (uart_idx);
                    printf ("len : %d\r\n", rec_buf_cnt);

                    if (rec_buf_cnt) {
                        CH9434UARTxGetRxFIFOData (uart_idx, uart_rec_buf, rec_buf_cnt);
                        CH9434_UART_FIFO_SEND (uart_idx, uart_rec_buf, rec_buf_cnt);
                    }

                    break;
                }
                case 0x0C:  // Data reception timeout
                {
                    uart_lsr = CH9434UARTxReadLSR (uart_idx);
                    printf ("lsr:%02x\r\n", uart_lsr);
                    rec_buf_cnt = CH9434UARTxGetRxFIFOLen (uart_idx);
                    printf ("len : %d\r\n", rec_buf_cnt);
                    if (rec_buf_cnt) {
                        CH9434UARTxGetRxFIFOData (uart_idx, uart_rec_buf, rec_buf_cnt);
                        CH9434_UART_FIFO_SEND (uart_idx, uart_rec_buf, rec_buf_cnt);
                    }

                    break;
                }
                case 0x02:  // THR register is empty
                    break;
                case 0x00:  // Changes in the modem signal
                {
                    uart_msr = CH9434UARTxReadMSR (uart_idx);
                    printf ("idx:%d uart_iir:%02x\r\n", uart_idx, uart_iir);
                    printf ("uart_msr:%02x\r\n", uart_msr);
                    break;
                }
                }
            }
        }
    }
}
