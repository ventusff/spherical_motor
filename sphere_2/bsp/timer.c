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

/**  ---------------------   ��ʱ��ռ�����  -------------------------------
 * TIM4   1Mhz      ���ж�        timer.c             ���ڲ���Nus��ʱ,Nms��ʱ.
 * TIM5   100khz    �ж�          bsp_motor_drive.c   ���ڲ���100khz��ʱ�жϡ�����ADC,DAC,Mouse Sensor
 * TIM2   1HZ       �ж�          timer.c              ���ڲ�����ȷ��1hzʱ�ӣ����Լ��ʱ���Ƿ�׼ȷ��ʱִ��
**/

/* ------------- ȫ�ֱ��� ------------------ */
void TIMER_Initializes(void)
{
  TIM4_Init();
	TIM2_Init();
}

void TIM2_Init(void)
{
  //ʱ������
  {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* RCCʱ������ */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TIM4ʱ����Ԫ���� */
    TIM_TimeBaseStructure.TIM_Prescaler = TIM2_PRESCALER_VALUE;        //Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = TIM2_PERIOD_TIMING;             //��ʱ����
    //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //ʱ�ӷ�Ƶ����
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* ʹ��Ԥ��Ƶֵ */
    TIM_ARRPreloadConfig(TIM2, ENABLE);                                //ʹ������ֵ
  }
  //�ж�����
  {
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ClearFlag(TIM2, TIM_FLAG_Update); 
    /* ��������ж� */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );
    
    /* ʹ��Ԥ��Ƶֵ */
    TIM_ARRPreloadConfig(TIM2, ENABLE);                                //ʹ������ֵ
    
    /* NVIC �ж����ȼ����� */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
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

  /* RCCʱ������ */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* TIM4ʱ����Ԫ���� */
  TIM_TimeBaseStructure.TIM_Prescaler = TIM4_PRESCALER_VALUE;        //Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period = TIM4_PERIOD_TIMING;             //��ʱ����
  //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //ʱ�ӷ�Ƶ����
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* ʹ��Ԥ��Ƶֵ */
  TIM_ARRPreloadConfig(TIM4, ENABLE);                                //ʹ������ֵ
}

void TIMDelay_Nus(uint16_t Times)
{
  TIM_Cmd(TIM4, ENABLE);                                             //������ʱ��
  while(Times--)
  {
    while(TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == RESET)
		{
			if(_cfg_PUT_MAIN_IN_SECONDARY)
				MainClockTask();
		};        //�ȴ��������
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);                            //�����־
  }
  TIM_Cmd(TIM4, DISABLE);                                            //������ʱ��
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
