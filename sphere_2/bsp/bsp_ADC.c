#include "bsp_ADC.h"
/*
*********************************************************************************************************
*
*	模块名称 : AD7606数据采集模块
*	文件名称 : bsp_adc.c
*	版    本 : V1.0
*	说    明 : AD7606挂在STM32的FSMC总线上。
*
*			本例子使用了 TIM4 作为硬件定时器，定时启动ADC转换
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/


/* 设置过采样的GPIO: PH9 PH10 PH11 */
/*
#define OS0_1()		GPIOH->BSRRL = GPIO_Pin_9
#define OS0_0()		GPIOH->BSRRH = GPIO_Pin_9
#define OS1_1()		GPIOH->BSRRL = GPIO_Pin_10
#define OS1_0()		GPIOH->BSRRH = GPIO_Pin_10
#define OS2_1()		GPIOH->BSRRL = GPIO_Pin_11
#define OS2_0()		GPIOH->BSRRH = GPIO_Pin_11
*/

static void AD7606_CtrlLinesConfig(void);
static void AD7606_FSMCConfig(void);

AD7606_VAR_T* p_tAD7606;
AD7606_readings* p_tReadings;

/*
*********************************************************************************************************
*	函 数 名: bsp_InitExtSRAM
*	功能说明: 配置连接外部SRAM的GPIO和FSMC
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAD7606(AD7606_VAR_T * _p_tAD7606,AD7606_readings * _p_tReadings)
{
  p_tAD7606 = _p_tAD7606;
  p_tReadings = _p_tReadings;
	AD7606_CtrlLinesConfig();
	AD7606_FSMCConfig();

	//AD7606_SetOS(AD_OS_NO);		/* 无过采样 */
	//AD7606_SetInputRange(0);	/* 0表示输入量程为正负5V, 1表示正负10V */

	AD7606_Reset();

	AD7606_CONV_1_H();					/* 启动转换的GPIO平时设置为高 */
  AD7606_CONV_2_H();
	AD7606_CONV_3_H();
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_CtrlLinesConfig
*	功能说明: 配置LCD控制口线，FSMC管脚设置为复用功能
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
/*
	STM32开发板接线方法：

	PD0/FSMC_D2
	PD1/FSMC_D3
	//PD4/FSMC_NOE		--- 读控制信号，OE = Output Enable ， N 表示低有效
		//因为采用手册中 CS 与 RD 直接相连的方式，因此此处也不需要RD信号了
	//PD5/FSMC_NWE		--- 写控制信号，AD7606 只有读，无写信号
	PD8/FSMC_D13
	PD9/FSMC_D14
	PD10/FSMC_D15

	PD14/FSMC_D0
	PD15/FSMC_D1

	PE4/FSMC_A20		
	PE5/FSMC_A21		
	
	PE2/FSMC_A23		--- 和主片选一起译码
	PG13/FSMC_A24		--- 和主片选一起译码
	PG14/FMSC_A25		--- 和主片选一起译码
	
	PE7/FSMC_D4
	PE8/FSMC_D5
	PE9/FSMC_D6
	PE10/FSMC_D7
	PE11/FSMC_D8
	PE12/FSMC_D9
	PE13/FSMC_D10
	PE14/FSMC_D11
	PE15/FSMC_D12

	PG12/FSMC_NE4		--- 主片选（TFT, OLED 和 AD7606）

	其他的控制IO:

	PH9/DCMI_D0/AD7606_OS0			---> AD7606_OS0		OS2:OS0 选择数字滤波参数
	PH10/DCMI_D1/AD7606_OS1         ---> AD7606_OS1
	PH11/DCMI_D2/AD7606_OS2         ---> AD7606_OS2
	PH12/DCMI_D3/AD7606_CONVST      ---> AD7606_CONVST	启动ADC转换 (CONVSTA 和 CONVSTB 已经并联)
	PH14/DCMI_D4/AD7606_RAGE        ---> AD7606_RAGE	输入模拟电压量程，正负5V或正负10V
	PI4/DCMI_D5/AD7606_RESET        ---> AD7606_RESET	复位
	PI6/DCMI_D6/AD7606_BUSY         ---> AD7606_BUSY	忙信号	

*/
static void AD7606_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能FSMC时钟 */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	/* 使能 GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE);

	/* 设置 PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) 为复用推挽输出 */

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
	                            GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*
		PE4/FSMC_A20		--- 和主片选一起译码
		PE5/FSMC_A21		--- 和主片选一起译码
		PE6/FSMC_A22
		PE2/FSMC_A23		--- 和主片选一起译码

		PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 	PE.14(D11), PE.15(D12)
	*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource2 , GPIO_AF_FSMC);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | 
															GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                            GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                            GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*
		PG13/FSMC_A24		--- 和主片选一起译码
		PG14/FMSC_A25		--- 和主片选一起译码
	*/
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	

	/* 设置 PG12  为复用推挽输出 */ //FSMC_NE4，主片选
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/*	配置几个控制用的GPIO
		PG6/AD7606_CONVST1      ---> AD7606_CONVST1	启动ADC转换1
		PG7/AD7606_CONVST2      ---> AD7606_CONVST2	启动ADC转换2
		PG8/AD7606_CONVST3      ---> AD7606_CONVST3	启动ADC转换2
		PG11/AD7606_RESET        ---> AD7606_RESET	复位
		//以下3个在中断处配置
		PA8/AD7606_BUSY1			---> AD7606_BUSY1    转换结束的信号
		PB8/AD7606_BUSY2			---> AD7606_BUSY2    转换结束的信号
		PB9/AD7606_BUSY3			---> AD7606_BUSY3    转换结束的信号
	*/
	{
		/* 使能 GPIOG时钟 */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_11;
		GPIO_Init(GPIOG, &GPIO_InitStructure);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_FSMCConfig
*	功能说明: 配置FSMC并口访问时序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AD7606_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  init;
	FSMC_NORSRAMTimingInitTypeDef  timing;

	/*
		AD7606规格书要求(3.3V时)：RD读信号低电平脉冲宽度最短21ns，高电平脉冲最短宽度15ns。

		按照如下配置 读数均正常。为了和同BANK的LCD配置相同，选择3-0-6-1-0-0
		3-0-5-1-0-0  : RD高持续75ns， 低电平持续50ns.  1us以内可读取8路样本数据到内存。
		1-0-1-1-0-0  : RD高75ns，低电平执行12ns左右，下降沿差不多也12ns.  数据读取正确。
	*/
	/* FSMC_Bank1_NORSRAM4 configuration */
	
	//实测1-0-1-1-0-0出现了channel混乱
	timing.FSMC_AddressSetupTime = 3;
	timing.FSMC_AddressHoldTime = 0;
	timing.FSMC_DataSetupTime = 5;
	timing.FSMC_BusTurnAroundDuration = 1;
	timing.FSMC_CLKDivision = 0;
	timing.FSMC_DataLatency = 0;
	timing.FSMC_AccessMode = FSMC_AccessMode_A;

	/*
	 LCD configured as follow:
	    - Data/Address MUX = Disable
	    - Memory Type = SRAM
	    - Data Width = 16bit
	    - Write Operation = Enable
	    - Extended Mode = Enable
	    - Asynchronous Wait = Disable
	*/
	init.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	init.FSMC_WrapMode = FSMC_WrapMode_Disable;
	init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	init.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	init.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

	init.FSMC_ReadWriteTimingStruct = &timing;
	init.FSMC_WriteTimingStruct = &timing;

	FSMC_NORSRAMInit(&init);

	/* - BANK 1 (of NOR/SRAM Bank 1~4) is enabled */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_SetOS
*	功能说明: 配置AD7606数字滤波器，也就设置过采样倍率。
*			 通过设置 AD7606_OS0、OS1、OS2口线的电平组合状态决定过采样倍率。
*			 启动AD转换之后，AD7606内部自动实现剩余样本的采集，然后求平均值输出。
*
*			 过采样倍率越高，转换时间越长。
*			 无过采样时，AD转换时间 4us;
*				2倍过采样时 = 8.7us;
*				4倍过采样时 = 16us
*			 	64倍过采样时 = 286us
*
*	形    参: _ucOS : 过采样倍率
*	返 回 值: 无
*********************************************************************************************************
*/
/*
void AD7606_SetOS(AD7606_OS_E _ucOS)
{
	g_tAD7606.ucOS = _ucOS;
	switch (_ucOS)
	{
		case AD_OS_X2:
			OS2_0();
			OS1_0();
			OS0_1();
			break;

		case AD_OS_X4:
			OS2_0();
			OS1_1();
			OS0_0();
			break;

		case AD_OS_X8:
			OS2_0();
			OS1_1();
			OS0_1();
			break;

		case AD_OS_X16:
			OS2_1();
			OS1_0();
			OS0_0();
			break;

		case AD_OS_X32:
			OS2_1();
			OS1_0();
			OS0_1();
			break;

		case AD_OS_X64:
			OS2_1();
			OS1_1();
			OS0_0();
			break;

		case AD_OS_NO:
		default:
			g_tAD7606.ucOS = AD_OS_NO;
			OS2_0();
			OS1_0();
			OS0_0();
			break;
	}
}
*/


/*
*********************************************************************************************************
*	函 数 名: AD7606_SetInputRange
*	功能说明: 配置AD7606模拟信号输入量程。
*	形    参: _ucRange : 0 表示正负5V   1表示正负10V
*	返 回 值: 无
*********************************************************************************************************
*/
/*
void AD7606_SetInputRange(uint8_t _ucRange)
{
	if (_ucRange == 0)
	{
		p_tAD7606->ucRange = 0;
		AD7606_RANGE_L();	// 设置为正负5V 
	}
	else
	{
		p_tAD7606->ucRange = 1;
		AD7606_RANGE_H();	// 设置为正负10V 
	}
}
*/

/*
*********************************************************************************************************
*	函 数 名: AD7606_Reset
*	功能说明: 硬件复位AD7606。复位之后恢复到正常工作状态。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_Reset(void)
{
	AD7606_RESET_L();	/* 退出复位状态 */

	AD7606_RESET_H();	/* 进入复位状态 */
	AD7606_RESET_H();	/* 仅用于延迟。 RESET复位高电平脉冲宽度最小50ns。 */
	AD7606_RESET_H();
	AD7606_RESET_H();

	AD7606_RESET_L();	/* 退出复位状态 */
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartConvst
*	功能说明: 启动1次ADC转换
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartConvst(uint8_t ADC_port)
{
	/* page 7：  CONVST 高电平脉冲宽度和低电平脉冲宽度最短 25ns */
	/* CONVST平时为高 */
	switch(ADC_port)
	{
		case AD7606_PORT1:
			{
				//实测200ns
				AD7606_CONV_1_L();
				AD7606_CONV_1_L();
				AD7606_CONV_1_L();
				AD7606_CONV_1_L();
				
				AD7606_CONV_1_H();
				break;
			}
		case AD7606_PORT2:
			{
				AD7606_CONV_2_L();
				AD7606_CONV_2_L();
				AD7606_CONV_2_L();
				AD7606_CONV_2_L();
				
				AD7606_CONV_2_H();
				break;
			}
		case AD7606_PORT3:
			{
				AD7606_CONV_3_L();
				AD7606_CONV_3_L();
				AD7606_CONV_3_L();
				AD7606_CONV_3_L();
				
				AD7606_CONV_3_H();
				break;
			}
		default:
		{
			break;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadNowAdc
*	功能说明: 读取8路采样结果。结果存储在全局变量 g_tAD7606
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_ReadNowAdc(uint8_t ADC_port)
{
    uint8_t idx = 0;
    for(idx = 0;idx<p_tAD7606->ucADchannels;idx++)
    {
		switch(ADC_port)
		{
			case AD7606_PORT1:
			{
				p_tAD7606->sNowAdc[idx] = AD7606_RESULT_1();	/* 读第idx+1路样本 */
				p_tReadings->RAWarray[ADC_port][idx] = p_tAD7606->sNowAdc[idx];
				break;
			}
			case AD7606_PORT2:
			{
				p_tAD7606->sNowAdc[idx] = AD7606_RESULT_2();	/* 读第idx+1路样本 */
				p_tReadings->RAWarray[ADC_port][idx] = p_tAD7606->sNowAdc[idx];
				break;
			}
			case AD7606_PORT3:
			{
				p_tAD7606->sNowAdc[idx] = AD7606_RESULT_3();	/* 读第idx+1路样本 */
				p_tReadings->RAWarray[ADC_port][idx] = p_tAD7606->sNowAdc[idx];
				break;
			}
		}
        
    }
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
