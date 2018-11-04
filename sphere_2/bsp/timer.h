
/* 定义防止递归包含 ----------------------------------------------------------*/
#ifndef _TIMER_H
#define _TIMER_H


/* 包含的头文件 --------------------------------------------------------------*/
#include "stm32f4xx.h"


/* 宏定义 --------------------------------------------------------------------*/
//TIM4 == 84MHZ /12 /7 ==1MHZ
#define TIM4_PRESCALER_VALUE      (1 - 1)
#define TIM4_PERIOD_TIMING        (84 - 1)                 
//TIM2 == 84MHZ /12000 / 7000 = 1HZ
#define TIM2_PRESCALER_VALUE      (12000 - 1)
#define TIM2_PERIOD_TIMING        (7000 - 1)                 


/* 函数申明 ------------------------------------------------------------------*/
void TIMER_Initializes(void);

//TIM4: 软件延时时钟
void TIM4_Init(void);
void TIMDelay_Nus(uint16_t Times);
void TIMDelay_Nms(uint16_t Times);

//TIM2: 时序检查时钟
void TIM2_Init(void);
void TimingCheckStart(void);
void TimingCheckStop(void);





#endif /* _TIMER_H */

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
