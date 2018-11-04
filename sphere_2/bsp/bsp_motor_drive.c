/**
 * 提供ADC,DAC的读取、写入驱动、接口
 * 
 * */
#include "bsp_motor_drive.h"

extern void MainClockTask(void);


AD7606_VAR_T g_tAD7606;		/* 定义1个全局变量，保存一些参数 */
AD7606_FIFO_T g_tAdcFifo[3];	/* 定义FIFO结构体变量 */
AD7606_readings g_tReadings;
volatile CoilDriveFLAGS g_tCoilDriveFLAGS;
//volatile indicates that a variable can be changed by a backgroud routine.
//只有加入这句话，main() 对 g_tCoilDriveFLAGS的修改才能生效.

/*
*********************************************************************************************************
*		下面的函数用于定时采集模式。 TIM5硬件定时中断中读取ADC结果，存在全局FIFO
*
*
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: AD7606_EnterAutoMode
*	功能说明: 配置硬件工作在自动采集模式，结果存储在FIFO缓冲区。
*	形    参：_ulFreq : 采样频率，单位Hz，	1k，2k，5k，10k，20K，50k，100k，200k
*	返 回 值: 无
*********************************************************************************************************
*/
/*
	最终决定：只有AD7606_1使用中断方式。
	其他均使用定时扫描方式。
	如想要读取频率为1000Hz，则可设置扫描频率为2000Hz；
	或者都共用AD7606_1的定时中断，只不过有分频。
	主要是发现：如果2个AD7606设置为同一个中断函数，则由于都使用EXTI_Line_9_5中断，中断会被吞掉。
*/
/*
	后面可以尝试修改主控板电路，保证3个BUSY引脚均可以区分中断的中断通道
*/
void AD7606_EnterAutoMode(uint32_t _ulFreq)
{
    g_tAD7606.ucADchannels = 8;
    bsp_InitAD7606(&g_tAD7606,&g_tReadings);

    //配置TIM5为ADC自动时钟
	{
		uint32_t uiTIMxCLK;
		uint16_t usPrescaler;
		uint16_t usPeriod;

		//TIM_DeInit(TIM5);	/* 复位TIM定时器 */

	    /*-----------------------------------------------------------------------
			system_stm32f4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

			HCLK = SYSCLK / 1     (AHB1Periph)
			PCLK2 = HCLK / 2      (APB2Periph)
			PCLK1 = HCLK / 4      (APB1Periph)

			因为APB1 prescaler != 1, 所以 APB1上的TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
			因为APB2 prescaler != 1, 所以 APB2上的TIMxCLK = PCLK2 x 2 = SystemCoreClock;

			APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
			APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11
		*/

		uiTIMxCLK = SystemCoreClock / 2;//168000000/2

		if (_ulFreq < 3000)
		{
			usPrescaler = 100 - 1;					/* 分频比 = 10 */
			usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* 自动重装的值 */
		}
		else	/* 大于4K的频率，无需分频 */
		{
			usPrescaler = 0;					/* 分频比 = 1 */
			usPeriod = uiTIMxCLK / _ulFreq - 1;	/* 自动重装的值 */
		}


        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        RCC_APB1PeriphClockCmd(motor_drive_clock_cmd, ENABLE);//使能TIM2，开TIM2的时钟

        TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;        //预分频值
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //向上计数模式
        TIM_TimeBaseStructure.TIM_Period = usPeriod;             //定时周期
        //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //时钟分频因子——基本定时器无此功能
        TIM_TimeBaseInit(motor_drive_clock, &TIM_TimeBaseStructure);
        
    }

    //配置TIM5的中断为开始一次
  {
				NVIC_InitTypeDef NVIC_InitStructure;
        TIM_ClearFlag(motor_drive_clock, TIM_FLAG_Update); 
        /* 允许更新中断 */
        TIM_ITConfig(motor_drive_clock,TIM_IT_Update,ENABLE );
        
        /* 使能预分频值 */
        TIM_ARRPreloadConfig(motor_drive_clock, ENABLE);                                //使能重载值
        
        /* NVIC 中断优先级设置 */
        NVIC_InitStructure.NVIC_IRQChannel = motor_drive_irqn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
        NVIC_Init(&NVIC_InitStructure); 
        
	}

	/* 配置PA8, BUSY 作为中断输入口，下降沿触发 */
	if(_cfg_USE_ADC_1)
	{
		EXTI_InitTypeDef   EXTI_InitStructure;
		GPIO_InitTypeDef   GPIO_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Configure PA8 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
			/* Connect EXTA Line8 to PI6 pin */
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

			/* Configure EXTA Line8 */
			EXTI_InitStructure.EXTI_Line = EXTI_Line8;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

			//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);

			/* Enable and set EXTI Line8 Interrupt to the lowest priority */
			NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
	}
	if(_cfg_USE_ADC_2)
	{
		GPIO_InitTypeDef   GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		/* Configure PB8 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	if(_cfg_USE_ADC_3)
	{
		GPIO_InitTypeDef   GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		/* Configure PB9 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ISR
*	功能说明: 定时采集中断服务程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_ISR(uint8_t ADC_port)
{
	uint8_t i;

	AD7606_ReadNowAdc(ADC_port);

	//将结果写入FIFO，并更新usWrite,Count值
	for (i = 0; i < 8; i++)
	{
		g_tAdcFifo[ADC_port].sBuf[g_tAdcFifo[ADC_port].usWrite][i] = g_tAD7606.sNowAdc[i];
	}

	if (++g_tAdcFifo[ADC_port].usWrite >= ADC_FIFO_SIZE)
	{
		g_tAdcFifo[ADC_port].usWrite = 0;
	}
	if (g_tAdcFifo[ADC_port].usCount < ADC_FIFO_SIZE)
	{
		g_tAdcFifo[ADC_port].usCount++;
		g_tAdcFifo[ADC_port].ucFull = 0;
	}
	else
	{
		g_tAdcFifo[ADC_port].ucFull = 1;		/* FIFO 满，主程序来不及处理数据 */
	}

}

void MainClockISR(void)
{
	static uint16_t COUNT_DIVIDE = 1;
	if(_cfg_USE_ADC_1)
	{
		AD7606_StartConvst(AD7606_PORT1);
	}
		
	


	//只有每个100分频的最开始开始转换
	if(COUNT_DIVIDE == 1)
	{
		if(_cfg_USE_ADC_2)
			AD7606_StartConvst(AD7606_PORT2);
		if(_cfg_USE_ADC_3)
			AD7606_StartConvst(AD7606_PORT3);
		g_tCoilDriveFLAGS.AD7606_2_PREVIOUS = 1;
		g_tCoilDriveFLAGS.AD7606_3_PREVIOUS = 1;
	}
	else if(COUNT_DIVIDE >= _cst_MINIMUM_CLOCK_CYCLE_2_3)
	{
		//检查AD7606_2,AD7606_3的ready状态
		if(_cfg_USE_ADC_2)
		{
			if(AD7606_2_READY())
			{
				if(g_tCoilDriveFLAGS.AD7606_2_PREVIOUS == 0)
				{
					AD7606_ISR(AD7606_PORT2);
					g_tCoilDriveFLAGS.FLAG_READY_2 = 1;
				}
				g_tCoilDriveFLAGS.AD7606_2_PREVIOUS = 1;
			}
			else
			{
				g_tCoilDriveFLAGS.AD7606_2_PREVIOUS = 0;
			}
		}
		
		if(_cfg_USE_ADC_3)
		{
			if(AD7606_3_READY())
			{
				if(g_tCoilDriveFLAGS.AD7606_3_PREVIOUS == 0)
				{
					
					AD7606_ISR(AD7606_PORT3);
					g_tCoilDriveFLAGS.FLAG_READY_3 = 1;
				}
				g_tCoilDriveFLAGS.AD7606_3_PREVIOUS = 1;
			}
			else
			{
				g_tCoilDriveFLAGS.AD7606_3_PREVIOUS = 0;
			}
		}
	}
	else
	{
		if(_cfg_USE_ADC_2)
			if(!AD7606_2_READY())
				g_tCoilDriveFLAGS.AD7606_2_PREVIOUS = 0;
		if(_cfg_USE_ADC_3)
			if(!AD7606_3_READY())
				g_tCoilDriveFLAGS.AD7606_3_PREVIOUS = 0;
	}

	if(COUNT_DIVIDE == _cst_MAIN_DIVIDE_TO_SECONDARY)
	{
		COUNT_DIVIDE = 1;
	}
	else
	{
		COUNT_DIVIDE++;
	}
}

//TIM5定时中断，开始AD7606的转换
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(motor_drive_clock, TIM_IT_Update) != RESET) 
	{
		MainClockISR();
		TIM_ClearITPendingBit(motor_drive_clock, TIM_IT_Update ); 
	}
	
}

/*
由于一些尴尬操作，AD7606_2使用PB8, AD7606_1使用PA8
但是不能同时给他们设置中断。
这样，就只能给AD7606_2单独使用一套定时扫描来读取的方式。
AD7606_3可以继续采用定时读取的方式；
AD7606_2也建议继续采用定时读取的方式，主要是为了保证时序一直有效
*/
/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序入口。PH12/AD7606_BUSY 下降沿中断触发
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		AD7606_ISR(AD7606_PORT1);
		g_tCoilDriveFLAGS.FLAG_READY_1 = 1;
		//_tim_TickMainTimer();
		
		//VENTUSFF: 直接放在中断中！
		//考虑到目前20khz，50us一个周期，足够完成20us的DAC写；
		//考虑到本意就是要这样；
		//MainClockTask();
		if(_cfg_PUT_MAIN_IN_SECONDARY)
		{
			if(g_tCoilDriveFLAGS.MainPID_Calculating)
			{
				MainClockTask();
			}
		}


		/* Clear the EXTI line 8 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_HasNewData
*	功能说明: 判断FIFO中是否有新数据
*	形    参：_usReadAdc : 存放ADC结果的变量指针
*	返 回 值: 1 表示有，0表示暂无数据
*********************************************************************************************************
*/
uint8_t AD7606_HasNewData(uint8_t ADC_port)
{
	if (g_tAdcFifo[ADC_port].usCount > 0)
	{
		return 1;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_FifoFull
*	功能说明: 判断FIFO是否满
*	形    参：_usReadAdc : 存放ADC结果的变量指针
*	返 回 值: 1 表示满，0表示未满
*********************************************************************************************************
*/
uint8_t AD7606_FifoFull(uint8_t ADC_port)
{
	return g_tAdcFifo[ADC_port].ucFull;
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadFifo
*	功能说明: 从FIFO中读取一个ADC值
*	形    参：_usReadAdc_x8 : 存放ADC结果的变量指针,8变量数组
*	返 回 值: 1 表示OK，0表示暂无数据
*********************************************************************************************************
*/
uint8_t AD7606_ReadFifo(uint8_t ADC_port, int16_t *_usReadAdc_x8)
{
	if (AD7606_HasNewData(ADC_port))
	{
		for(uint8_t i = 0;i<8;i++)
		{
			_usReadAdc_x8[i] = g_tAdcFifo[ADC_port].sBuf[g_tAdcFifo[ADC_port].usRead][i];
		}
		
		if (++g_tAdcFifo[ADC_port].usRead >= ADC_FIFO_SIZE)
		{
			g_tAdcFifo[ADC_port].usRead = 0;
		}

		//DISABLE_INT();
		/*
		#define ENABLE_INT()	__set_PRIMASK(0)	// 使能全局中断 
		#define DISABLE_INT()	__set_PRIMASK(1)	// 禁止全局中断 
		*/
		
		if (g_tAdcFifo[ADC_port].usCount > 0)
		{
			g_tAdcFifo[ADC_port].usCount--;
		}
		//ENABLE_INT();
		return 1;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartRecord
*	功能说明: 开始采集
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartRecord(uint8_t ADC_port, uint32_t _ulFreq)
{
	AD7606_StopRecord();

	AD7606_Reset();					/* 复位硬件 */
	AD7606_StartConvst(ADC_port);			/* 启动采样，避免第1组数据全0的问题 */
	
	g_tAdcFifo[ADC_port].usRead = 0;			/* 必须在开启TIM2之前清0 */
	g_tAdcFifo[ADC_port].usWrite = 0;
	g_tAdcFifo[ADC_port].usCount = 0;
	g_tAdcFifo[ADC_port].ucFull = 0;

	AD7606_EnterAutoMode(_ulFreq);
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StopRecord
*	功能说明: 停止采集定时器
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StopRecord(void)
{
	TIM_Cmd(TIM5, DISABLE);

    //下面这段专用于将PH12专用于PWM输出时（其实只是因为PWM输出口恰好和原来定义的CONVST口冲突）
	/* 将PH12 重新配置为普通输出口 */
    /*
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		// 使能 GPIO时钟 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOH, &GPIO_InitStructure);
	}
    */

	AD7606_CONV_1_H();					/* 启动转换的GPIO平时设置为高 */	\
  AD7606_CONV_2_H();
	AD7606_CONV_3_H();
}

void _bsp_StartMainClock(void)
{
	/* 使能motor_drive_clock */
    TIM_Cmd(motor_drive_clock, ENABLE);
}

void _bsp_StopMainClock(void)
{
	/* 使能motor_drive_clock */
    TIM_Cmd(motor_drive_clock, DISABLE);
}

/******************************  DAC8563 相关调用区   *********************************/

/******************************  总工作区   *********************************/
void bsp_motor_drive_initializes(void)
{
	printx("Initiazling AD7606 & DAC8563...");
	g_tCoilDriveFLAGS.AD7606_2_PREVIOUS = 0;
	g_tCoilDriveFLAGS.AD7606_3_PREVIOUS = 0;
	g_tCoilDriveFLAGS.FLAG_READY_1 = 0;
	g_tCoilDriveFLAGS.FLAG_READY_2 = 0;
	g_tCoilDriveFLAGS.FLAG_READY_3 = 0;
	g_tCoilDriveFLAGS.MainPID_Calculating = 0;
	AD7606_EnterAutoMode(_cfg_MAIN_CLOCK);
	bsp_InitDAC8562();
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
