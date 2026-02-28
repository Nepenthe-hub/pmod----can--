/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2025/08/08
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co.Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "SPI_Init.h"
#include "IIC_Init.h"
#include "CH9434.H"


/* CAN Mode Definition */
#define TX_MODE 0
#define RX_MODE 1

/* Frame Format Definition */
#define Standard_Frame 0
#define Extended_Frame 1

/* CAN Communication Mode Selection */
#define CAN_MODE TX_MODE
// #define CAN_MODE RX_MODE

/* Frame Formate Selection */
#define Frame_Format Standard_Frame
// #define Frame_Format   Extended_Frame
/* Global Variable */


/* Receive can data in interrupt */
// #define USE_INTERRUPT

#define CANSOFTFILTER_MAX_GROUP_NUM 2  // The maximum recommended configuration is 14.
                                       // Configure only what you need to prevent excessive RAM usage or an increase in the software's filtering time.

#define CANSOFTFILER_PREDEF_CTRLBYTE_MASK32 ((CAN_FilterScale_32bit << 5) | (CAN_FilterMode_IdMask << 1))
#define CANSOFTFILER_PREDEF_CTRLBYTE_ID32 ((CAN_FilterScale_32bit << 5) | (CAN_FilterMode_IdList << 1))

/*
This is the structure of the software filtering table. It can be configured through the CAN_SoftFilterInit function,
or you can directly set the configuration values. The configured values can be modified directly during runtime.
However, when using the interrupt mode for reception, you need to be aware that if the modification is interrupted,
it may affect the filtering results during this period.
*/

struct CH9434D_CANFilterStruct_t {
    union {
        union {
            struct
            {
                uint32_t : 1;
                uint32_t RTR : 1;
                uint32_t IDE : 1;
                uint32_t ExID : 29;
            } Access_Ex;

            struct
            {
                uint32_t : 1;
                uint32_t RTR : 1;
                uint32_t IDE : 1;
                uint32_t : 18;
                uint32_t StID : 11;
            } Access_St;
        };

        union {
            struct {
                uint16_t FR_16_L;
                uint16_t FR_16_H;
            };

            uint32_t FR_32;
        };
    } FR[2];

    union {
        struct
        {
            uint16_t en : 1;
            uint16_t mode : 4;
            uint16_t scale : 3;
        };

        uint16_t ctrl_byte;
    };
} CH9434D_CANFilterStruct[CANSOFTFILTER_MAX_GROUP_NUM];

uint8_t interrupt_rx_flag = 0;
volatile u8 canexbuf_interrupt[8];

void CH9434D_CAN_SoftFilterInit (CH9434D_CAN_FilterInitTypeDef *CAN_FilterInitStruct);
void CH9434D_CAN_Test_Mode_Init (u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
void CH9434D_CAN_ReceiveViaSoftFilter (uint8_t FIFONumber, CH9434D_CanRxMsg *RxMessage);
u8 CH9434D_CAN_Send_Msg (u8 *msg, u8 len);
u8 CH9434D_CAN_Receive_Msg (u8 *buf);

extern void CH9434WriteCANReg (u8_t reg_add, u32_t reg_val);
extern u32_t CH9434ReadCANReg (u8_t reg_add);

void CH9434D_CAN_RX_IRQHandler() {
    uint8_t px, pbuf[8];
    if (CH9434D_CAN_GetITStatus (CH9434D_CAN_IT_FMP0)) {
        px = CH9434D_CAN_Receive_Msg (pbuf);
        for (int i = 0; i < px; i++) {
            canexbuf_interrupt[i] = pbuf[i];
        }
        if (px) {
            interrupt_rx_flag = 1;
        }
        CH9434D_CAN_ClearITPendingBit (CH9434D_CAN_IT_FMP0);
    }
}

/*********************************************************************
 * @fn      CH9434D_CAN_SoftFilterInit
 *
 * @brief   Initializes the CAN peripheral according to the specified
 *        parameters in the CAN_FilterInitStruct.
 *
 * @param   CH9434D_CAN_FilterInitStruct - pointer to a CH9434D_CAN_FilterInitTypeDef
 *        structure that contains the configuration information.
 *
 * @return  none
 */
void CH9434D_CAN_SoftFilterInit (CH9434D_CAN_FilterInitTypeDef *CAN_FilterInitStruct) {
    if (CAN_FilterInitStruct->CAN_FilterNumber > sizeof (CH9434D_CANFilterStruct) / sizeof (*CH9434D_CANFilterStruct)) {
        return;
    }

    if (CAN_FilterInitStruct->CAN_FilterActivation) {
        CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].en = 1;
    } else {
        CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].en = 0;
    }
    CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].FR[0].FR_16_H = CAN_FilterInitStruct->CAN_FilterIdHigh;
    CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].FR[0].FR_16_L = CAN_FilterInitStruct->CAN_FilterIdLow;
    CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].FR[1].FR_16_H = CAN_FilterInitStruct->CAN_FilterMaskIdHigh;
    CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].FR[1].FR_16_L = CAN_FilterInitStruct->CAN_FilterMaskIdLow;
    CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].mode = CAN_FilterInitStruct->CAN_FilterMode;
    CH9434D_CANFilterStruct[CAN_FilterInitStruct->CAN_FilterNumber].scale = CAN_FilterInitStruct->CAN_FilterScale;
}

/*********************************************************************
 * @fn      CAN_ReceiveViaSoftFilter
 *
 * @brief   Receives a message via soft filter.
 *
 * @param   CANx - where x can be 1 to select the CAN peripheral.
 *          FIFONumber - Receive FIFO number.
 *            CAN_FIFO0.
 *          RxMessage -  pointer to a structure receive message which contains
 *        CAN Id, CAN DLC, CAN datas and FMI number.
 *
 * @return  none
 */
void CAN_ReceiveViaSoftFilter (uint8_t FIFONumber, CH9434D_CanRxMsg *RxMessage) {
    for (int group = 0; group < sizeof (CH9434D_CANFilterStruct) / sizeof (*CH9434D_CANFilterStruct); group++) {
        if (CH9434D_CANFilterStruct[group].en) {
            uint32_t temp = CH9434ReadCANReg (CH9434D_CAN_RXMIR0) & (~0x1);

            switch ((uint8_t)CH9434D_CANFilterStruct[group].ctrl_byte & ~0x1) {

            case CANSOFTFILER_PREDEF_CTRLBYTE_ID32:
                if ((CH9434D_CANFilterStruct[group].FR[0].FR_32 != temp) && (CH9434D_CANFilterStruct[group].FR[1].FR_32 != temp)) {
                    continue;
                } else {
                    CH9434D_CAN_Receive (CH9434D_CAN_FIFO0, RxMessage);
                    return;
                }
                break;

            case CANSOFTFILER_PREDEF_CTRLBYTE_MASK32:
                if ((CH9434D_CANFilterStruct[group].FR[0].FR_32 & CH9434D_CANFilterStruct[group].FR[1].FR_32) ^ (temp & CH9434D_CANFilterStruct[group].FR[1].FR_32)) {
                    continue;
                } else {
                    CH9434D_CAN_Receive (CH9434D_CAN_FIFO0, RxMessage);
                    return;
                }
                break;

            default:
                return;
                break;
            }
        }
    }
    CH9434D_CAN_FIFORelease (CH9434D_CAN_FIFO0);
}

/*********************************************************************
 * @fn      CH9434D_CAN_Mode_Init
 *
 * @brief   Initializes CAN communication test mode.
 *          Bps =Fpclk1/((tpb1+1+tbs2+1+1)*brp)
 *
 * @param   tsjw - CAN synchronisation jump width.
 *          tbs2 - CAN time quantum in bit segment 1.
 *          tbs1 - CAN time quantum in bit segment 2.
 *          brp - Specifies the length of a time quantum.
 *          mode - Test mode.
 *            CAN_Mode_Normal.
 *            CAN_Mode_LoopBack.
 *            CAN_Mode_Silent.
 *            CAN_Mode_Silent_LoopBack.
 *
 * @return  none
 */
void CH9434D_CAN_Mode_Init (u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode) {

    CH9434D_CAN_InitTypeDef CAN_InitStructure ={0};
    CH9434D_CAN_FilterInitTypeDef CAN_FilterInitStructure ={0};

    CAN_InitStructure.CAN_TTCM = DISABLE_T;
    CAN_InitStructure.CAN_ABOM = DISABLE_T;
    CAN_InitStructure.CAN_AWUM = DISABLE_T;
    CAN_InitStructure.CAN_NART = ENABLE_T;
    CAN_InitStructure.CAN_RFLM = DISABLE_T;
    CAN_InitStructure.CAN_TXFP = DISABLE_T;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;// Bps = Fpclk1/((tpb1+1+tbs2+1+1)*brp)

    CH9434D_CAN_Init (&CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 0;

#if (Frame_Format == Standard_Frame)
    CAN_FilterInitStructure.CAN_FilterMode = CH9434D_CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CH9434D_CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x62E0;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;

#elif (Frame_Format == Extended_Frame)
    /* identifier/mask mode, One 32-bit filter, ExtId: 0x12124567 */
    CAN_FilterInitStructure.CAN_FilterMode = CH9434D_CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CH9434D_CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x9092;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x2B3C;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFC;

#endif

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CH9434D_CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE_T;
    CH9434D_CAN_SoftFilterInit (&CAN_FilterInitStructure);

    CH9434D_CAN_ITConfig (CH9434D_CAN_IT_FMP0, ENABLE_T);
}


/*********************************************************************
 * @fn      CH9434D_CAN_Send_Msg
 *
 * @brief   CH9434D CAN Transmit function.
 *
 * @param   msg - Transmit data buffer.
 *          len - Data length.
 *
 * @return  0 - Send successful.
 *          1 - Send failed.
 */
u8 CH9434D_CAN_Send_Msg (u8 *msg, u8 len) {
    u8 mbox;
    u16 i = 0;

    CH9434D_CanTxMsg CanTxStructure;

#if (Frame_Format == Standard_Frame)
    CanTxStructure.StdId = 0x317;
    CanTxStructure.IDE = CH9434D_CAN_Id_Standard;

#elif (Frame_Format == Extended_Frame)
    CanTxStructure.ExtId = 0x12124567;
    CanTxStructure.IDE = CH9434D_CAN_Id_Extended;

#endif

    CanTxStructure.RTR = CH9434D_CAN_RTR_Data;
    CanTxStructure.DLC = len;

    for (i = 0; i < len; i++) {
        CanTxStructure.Data[i] = msg[i];
    }

    mbox = CH9434D_CAN_Transmit (&CanTxStructure);

    printf ("mbox = %02x\r\n", mbox);

    i = 0;

    while ((CH9434D_CAN_TransmitStatus (mbox) != CH9434D_CAN_TxStatus_Ok) && (i < 0xFFF)) {
        i++;
    }

    if (i == 0xFFF) {
        return 1;
    } else {
        return 0;
    }
}

/*********************************************************************
 * @fn      CH9434D_CAN_Receive_Msg
 *
 * @brief   CH9434D CAN Receive function.
 *
 * @param   buf - Receive data buffer.
 *          len - Data length.
 *
 * @return  CanRxStructure.DLC - Receive data length.
 */
u8 CH9434D_CAN_Receive_Msg (u8 *buf) {
    u8 i;

    CH9434D_CanRxMsg CanRxStructure;

    if (CH9434D_CAN_MessagePending (CH9434D_CAN_FIFO0) == 0) {
        return 0;
    }

    CH9434D_CAN_ReceiveViaSoftFilter (CH9434D_CAN_FIFO0, &CanRxStructure);

    for (i = 0; i < 8; i++) {
        buf[i] = CanRxStructure.Data[i];
    }

    return CanRxStructure.DLC;
}




/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main (void) {
     u8 i = 0;
    u8 cnt = 1;
    __unused u8 px;
    __unused u8 pxbuf[8];

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init (115200);
    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("This is the CAN sample project for CH9434D.\r\n");

#if (CAN_MODE == TX_MODE)
    printf ("Tx Mode\r\n");

#elif (CAN_MODE == RX_MODE)
    printf ("Rx Mode\r\n");

#endif

    Delay_Ms (500); 

#if ((USE_CHIP_MODEL == CHIP_MODEL_CH9434D) && (CH9434D_IF_SEL == CH9434D_IF_IIC))
    CH9434_IIC_INIT();
#elif ((USE_CHIP_MODEL == CHIP_MODEL_CH9434D) && (CH9434D_IF_SEL == CH9434D_IF_SPI))
    CH9434_SPI_Init();
#endif
    CH9434_INT_Init();

    CH9434OscXtFreqSet (16000000);
    CH9434DefIOFuncEn (5); 

    CH9434InitClkMode (CH9434_ENABLE, CH9434_ENABLE, 1);

    CH9434MulIOFuncEn (CH9434_GPIO_1);  
    CH9434DefIOFuncEn (CH9434_GPIO_6); 

    CH9434D_CAN_Mode_Init (CH9434D_CAN_SJW_tq (1), CH9434D_CAN_BS2_tq (5), CH9434D_CAN_BS1_tq (6), 16, CH9434D_CAN_Mode_Normal);

    printf ("CH9434D will fall asleep immediately\r\n");


    while (1) {
        #if (CAN_MODE == TX_MODE)
        for (i = 0; i < 8; i++) {
            pxbuf[i] = i;
        }

        px = CH9434D_CAN_Send_Msg (pxbuf, 8);

        if (px) {
            printf ("Send Failed\r\n");
        } else {
            printf ("Send Success\r\n");
            printf ("Send Data:\r\n");

            for (i = 0; i < 8; i++) {
                printf ("%02x\r\n", pxbuf[i]);
            }
        }
        Delay_Ms (500);
#elif (CAN_MODE == RX_MODE)

#ifndef USE_INTERRUPT
        px = CH9434D_CAN_Receive_Msg (pxbuf);

        if (px) {
            printf ("Receive Data:\r\n");

            for (i = 0; i < 8; i++) {
                printf ("%02x\r\n", pxbuf[i]);
            }
        }
        Delay_Ms (500);
#else
        if (interrupt_rx_flag) {
            interrupt_rx_flag = 0;
            printf ("Receive Data: ");
            for (i = 0; i < 8; i++) {
                printf ("%02x\t", canexbuf_interrupt[i]);
            }
            printf ("\n");
        }
#endif
#endif
        cnt++;
        if (cnt == 0xFF) {
            cnt = 0;
        }
    };
}
