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
    printf ("This is the low test program for CH9434.\r\n");

#if ((USE_CHIP_MODEL == CHIP_MODEL_CH9434D) && (CH9434D_IF_SEL == CH9434D_IF_IIC))
    CH9434_IIC_INIT();
#else
    CH9434_SPI_Init();
#endif

    CH9434_INT_Init();
    Delay_Ms (500);

#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
    CH9434InitClkMode (CH9434_ENABLE, CH9434_ENABLE, 13);
    Delay_Ms (10);
#elif (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)

    CH9434OscXtFreqSet (16000000);
    CH9434DefIOFuncEn (5);
    CH9434InitClkMode (CH9434_ENABLE, CH9434_ENABLE, 1);
    Delay_Ms (10);
    // After initializing all the I/O, perform the sleep operation to achieve the lowest power consumption.
    CH9434DefIOFuncEn (1);
    CH9434DefIOFuncEn (2);
    CH9434DefIOFuncEn (3);
    CH9434DefIOFuncEn (4);
    CH9434DefIOFuncEn (8);
    CH9434DefIOFuncEn (9);

    CH9434MulIOFuncEn (1);
    CH9434MulIOFuncEn (8);
#endif

    while (1) {
#if (USE_CHIP_MODEL == CHIP_MODEL_CH9434A)
        CH9434LowerPowerModeSet (1);
#elif (USE_CHIP_MODEL == CHIP_MODEL_CH9434D)
        CH9434LowerPowerModeSet (2);
#endif
        printf ("Sleep state\r\n");
        Delay_Ms (3000);
        CH9434WakeUp();
        printf ("Awakening state\r\n");
        Delay_Ms (3000);
        /*Attention! In the power-off mode, after waking up, CH9434 needs to be reinitialized.*/
    }
}