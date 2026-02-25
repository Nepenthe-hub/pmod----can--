#include "Timer.h"

/**
  * @brief  定时器2初始化，用于1秒定时中断
  */
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 开启TIM2时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 定时器配置：1秒中断一次 */
    /* 假设系统时钟72MHz, APB1时钟36MHz，经过2倍频后为72MHz */
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;              // 重装载值 (10ms * 100 = 1s)
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;            // 预分频器 (72MHz/7200 = 10kHz)
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* 开启更新中断 */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* NVIC配置 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE);
}