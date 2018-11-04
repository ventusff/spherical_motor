#include "timer.h"
#include "app_TimingCheck.h"

extern void MainClockTask(void);

/**
 * PRESCALER:  1-65536
 * COUNTER:    TIM2 & TIM5: 16-bit;    TIM3 & TIM4: 32-bit
*/

/**
 * APB1:42Mhz: Max Timer Clock:  84MHZ;    TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,  TIM12,TIM13,TIM14
 * APB2:84Mhz: Max Timer Clock:  168MHZ;   TIM1,TIM8,TIM9,TIM10,TIM11
**/

/**  ---------------------   定时器占用情况  -------------------------------
 * TIM4   1Mhz      无中断        timer.c             用于产生Nus延时,Nms延时.
 * TIM5   100khz    中断          bsp_motor_drive.c   用于产生100khz定时中断。用于ADC,DAC,Mouse Sensor
 * TIM2   1HZ       中断          timer.c              用于产生精确地1hz时钟，用以检查时序是否准确定时执行
**/

/* ------------- 全局变量 ------------------ */
void TIMER_Initializes(void)
{
  TIM4_Init();
	TIM2_Init();
}

void TIM2_Init(void)
{
  //时钟配置
  {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* RCC时钟配置 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TIM4时基单元配置 */
    TIM_TimeBaseStructure.TIM_Prescaler = TIM2_PRESCALER_VALUE;        //预分频值
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = TIM2_PERIOD_TIMING;             //定时周期
    //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //时钟分频因子
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* 使能预分频值 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);                                //使能重载值
  }
  //中断配置
  {
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ClearFlag(TIM2, TIM_FLAG_Update); 
    /* 允许更新中断 */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );
    
    /* 使能预分频值 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);                                //使能重载值
    
    /* NVIC 中断优先级设置 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); 
  }
}

void TimingCheckStart(void)
{
  _tim_ZeroAllTimer();
  TIM_Cmd(TIM2,ENABLE);
  _tim_TickGroundTruth();
}

void TimingCheckStop(void)
{
  TIM_Cmd(TIM2,DISABLE);
}

void TIM4_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* RCC时钟配置 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* TIM4时基单元配置 */
  TIM_TimeBaseStructure.TIM_Prescaler = TIM4_PRESCALER_VALUE;        //预分频值
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //向上计数模式
  TIM_TimeBaseStructure.TIM_Period = TIM4_PERIOD_TIMING;             //定时周期
  //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //时钟分频因子
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* 使能预分频值 */
  TIM_ARRPreloadConfig(TIM4, ENABLE);                                //使能重载值
}

void TIMDelay_Nus(uint16_t Times)
{
  TIM_Cmd(TIM4, ENABLE);                                             //启动定时器
  while(Times--)
  {
    while(TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == RESET)
		{
			if(_cfg_PUT_MAIN_IN_SECONDARY)
				MainClockTask();
		};        //等待计数完成
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);                            //清除标志
  }
  TIM_Cmd(TIM4, DISABLE);                                            //启动定时器
	if(_cfg_PUT_MAIN_IN_SECONDARY)
		MainClockTask();
}

void TIMDelay_Nms(uint16_t Times)
{
  while(Times--)
  {
    TIMDelay_Nus(1000);
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); 
    _tim_TickGroundTruth();
  }
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
