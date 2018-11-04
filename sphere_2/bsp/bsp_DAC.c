/*
*********************************************************************************************************
*
*	模块名称 : DAC8562/8563 驱动模块(单通道带16位DAC)
*	文件名称 : bsp_dac8562.c
*	版    本 : V1.0
*	说    明 : DAC8562/8563模块和CPU之间采用SPI接口。本驱动程序支持硬件SPI接口和软件SPI接口。
*			  通过宏切换。
*
*	修改记录 :
*		版本号  日期         作者     说明
*		V1.0    2014-01-17  armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp_DAC.h"

#define SOFT_SPI		/* 定义此行表示使用GPIO模拟SPI接口 */
//#define HARD_SPI		/* 定义此行表示使用CPU的硬件SPI接口 */

/*
	DAC8501模块可以直接插到STM32-V5开发板CN19排母(2*4P 2.54mm)接口上

    DAC8562/8563模块    STM32F407开发板
	  GND   ------  GND    
	  VCC   ------  3.3V
	  
	  LDAC  ------  PA4/NRF905_TX_EN/NRF24L01_CE/DAC1_OUT
      SYNC  ------  PF7/NRF24L01_CSN
      	  
      SCLK  ------  PB3/SPI3_SCK
      DIN   ------  PB5/SPI3_MOSI

			------  PB4/SPI3_MISO
	  CLR   ------  PH7/NRF24L01_IRQ
*/

/*
	DAC8562基本特性:
	1、供电2.7 - 5V;  【本例使用3.3V】
	4、参考电压2.5V，使用内部参考

	对SPI的时钟速度要求: 高达50MHz， 速度很快.
	SCLK下降沿读取数据, 每次传送24bit数据， 高位先传
*/

#if !defined(SOFT_SPI) && !defined(HARD_SPI)
 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif

#ifdef SOFT_SPI		/* 软件SPI */
	/* 定义GPIO端口 */
	#define RCC_SCLK 	RCC_AHB1Periph_GPIOF
	#define PORT_SCLK	GPIOF
	#define PIN_SCLK	GPIO_Pin_8
	
	#define RCC_DIN 	RCC_AHB1Periph_GPIOC
	#define PORT_DIN 	GPIOC
	#define PIN_DIN 	GPIO_Pin_4
	#define PIN_DIN2 	GPIO_Pin_5
	#define PIN_DIN3 	GPIO_Pin_8
	#define PIN_DIN4 	GPIO_Pin_9
	
	/* 片选 */
	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
	#define PORT_SYNC	GPIOF
	#define PIN_SYNC	GPIO_Pin_7

	/* LDAC, 可以不用 */
	#define RCC_LDAC	RCC_AHB1Periph_GPIOF
	#define PORT_LDAC	GPIOF
	#define PIN_LDAC	GPIO_Pin_6
	
	/* CLR, 可以不用 */
	#define RCC_CLR 	RCC_AHB1Periph_GPIOD
	#define PORT_CLR	GPIOD
	#define PIN_CLR 	GPIO_Pin_6

	/* 定义口线置0和置1的宏 */
	#define SYNC_0()	PORT_SYNC->BSRRH = PIN_SYNC
	#define SYNC_1()	PORT_SYNC->BSRRL = PIN_SYNC

	#define SCLK_0()	PORT_SCLK->BSRRH = PIN_SCLK
	#define SCLK_1()	PORT_SCLK->BSRRL = PIN_SCLK

	#define DIN_0()		PORT_DIN->BSRRH = PIN_DIN
	#define DIN_1()		PORT_DIN->BSRRL = PIN_DIN
	

	#define LDAC_0()	PORT_LDAC->BSRRH = PIN_LDAC
	#define LDAC_1()	PORT_LDAC->BSRRL = PIN_LDAC

	#define CLR_0()		PORT_CLR->BSRRH = PIN_CLR
	#define CLR_1()		PORT_CLR->BSRRL = PIN_CLR	
#endif

#ifdef HARD_SPI		/* 硬件SPI (未做) */
	;
#endif

/*
*********************************************************************************************************
*	函 数 名: bsp_InitDAC8562
*	功能说明: 配置STM32的GPIO和SPI接口，用于连接 DAC8562
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitDAC8562(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef SOFT_SPI
	SYNC_1();	/* SYNC = 1 */

	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_CLR, ENABLE);

	/* 配置几个推挽输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Pin = PIN_SCLK;
	GPIO_Init(PORT_SCLK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DIN | PIN_DIN2 | PIN_DIN3 | PIN_DIN4;
	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_SYNC;
	GPIO_Init(PORT_SYNC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_CLR;
	GPIO_Init(PORT_CLR, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PIN_LDAC;
	GPIO_Init(PORT_LDAC, &GPIO_InitStructure);
#endif

	/* Power up DAC-A and DAC-B */
	//DAC8562_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));
	DAC8562_WriteCmd_Parallel_4_Same((4 << 19) | (0 << 16) | (3 << 0));
	
	/* LDAC pin inactive for DAC-B and DAC-A  不使用LDAC引脚更新数据 */
	//DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));
	//DAC8562_WriteCmd_Parallel_4_Same((6 << 19) | (3 << 16) | (3 << 0));
	
	/* LDAC pin active for DAC-B and DAC-A  使用LDAC引脚更新数据 */
	DAC8562_WriteCmd_Parallel_4_Same((6 << 19) | (0 << 16) | (0 << 0));
	LDAC_1();	//硬件LDAC模式下，需要LDAC初始拉高
	

	/* 复位2个DAC到中间值, 输出2.5V */
	
	//DAC8562_SetData(0, 32767);
	//DAC8562_SetData(1, 32767);
	DAC8563_SetData_Simultaneously(32767,32767,32767,32767,32767,32767,32767,32767);
	//__NOP();//设置完数据与LDAC拉低之间，需要5ns,不过似乎不需要；
	LDAC_0();	//硬件LDAC模式下，写完数据需要LDAC拉低来同步数据

	/* 选择内部参考并复位2个DAC的增益=2 （复位时，内部参考是禁止的) */
	DAC8562_WriteCmd_Parallel_4_Same((7 << 19) | (0 << 16) | (1 << 0));
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd
*	功能说明: 向SPI总线发送24个bit数据。
*	形    参: _cmd : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_WriteCmd(uint32_t _cmd)
{
	uint8_t i;
	SYNC_0();
	
	/*　DAC8562 SCLK时钟高达50M，因此可以不延迟 */
	for(i = 0; i < 24; i++)
	{
		if (_cmd & 0x800000)
		{
			DIN_1();
		}
		else
		{
			DIN_0();
		}
		//实际检测可以发现，SCKL变高与变低之间大致持续了24ns
		SCLK_1();
		_cmd <<= 1;
		SCLK_0();
	}
	
	SYNC_1();
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd_Parallel_4
*	功能说明: 向4路DAC8563的SPI总线同时发送4路24个bit不同的数据。
*	形    参: _cmd : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_WriteCmd_Parallel_4(uint32_t _cmd1, uint32_t _cmd2, uint32_t _cmd3, uint32_t _cmd4)
{
	uint8_t i;
	SYNC_0();
	
	/*　DAC8562 SCLK时钟高达50M，因此可以不延迟 */
	uint32_t idx = 0x800000;
	for(i = 0; i < 24; i++)
	{
		uint16_t zeroPins = 0x0;
		uint16_t onePins = 0x0;
		
		if(_cmd1 & idx)
			onePins |= PIN_DIN;
		else
			zeroPins |= PIN_DIN;
		
		if(_cmd2 & idx)
			onePins |= PIN_DIN2;
		else
			zeroPins |= PIN_DIN2;
		
		if(_cmd3 & idx)
			onePins |= PIN_DIN3;
		else
			zeroPins |= PIN_DIN3;
		
		if(_cmd4 & idx)
			onePins |= PIN_DIN4;
		else
			zeroPins |= PIN_DIN4;
		
		if(onePins)
			GPIO_SetBits(PORT_DIN,onePins);
		if(zeroPins)
			GPIO_ResetBits(PORT_DIN,zeroPins);
		//实际检测可以发现，SCKL变高与变低之间大致持续了24ns
		SCLK_1();
		idx >>= 1;
		SCLK_0();
	}
	
	SYNC_1();
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd_Parallel_4
*	功能说明: 向4路DAC8563的SPI总线同时发送24bit相同的数据。
*	形    参: _cmd : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_WriteCmd_Parallel_4_Same(uint32_t _cmd)
{
	uint8_t i;
	uint16_t common = PIN_DIN | PIN_DIN2 | PIN_DIN3 | PIN_DIN4;
	SYNC_0();
	
	/*　DAC8562 SCLK时钟高达50M，因此可以不延迟 */
	for(i = 0; i < 24; i++)
	{
		if (_cmd & 0x800000)
		{
			GPIO_SetBits(PORT_DIN,common);
		}
		else
		{
			GPIO_ResetBits(PORT_DIN,common);
		}
		//实际检测可以发现，SCKL变高与变低之间大致持续了24ns
		SCLK_1();
		_cmd <<= 1;
		SCLK_0();
	}
	
	SYNC_1();
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_SetData
*	功能说明: 设置DAC输出，并立即更新。
*	形    参: _ch, 通道, 0 , 1
*		     _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void DAC8562_SetData(uint8_t _ch, uint16_t _dac)
{
	if (_ch == 0)
	{
		/* Write to DAC-A input register and update DAC-A; */
		DAC8562_WriteCmd((3 << 19) | (0 << 16) | (_dac << 0));
	}
	else if (_ch == 1)
	{
		/* Write to DAC-B input register and update DAC-A; */
		DAC8562_WriteCmd((3 << 19) | (1 << 16) | (_dac << 0));		
	}
}

void DAC8563_ZeroData_Simultaneously(void)
{
	DAC8563_SetData_Simultaneously(_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO,_cst_SUB_PID_OUTPUT_ZERO);
}

void DAC8563_SetData_Simultaneously(uint16_t _dac11, uint16_t _dac12, uint16_t _dac21, uint16_t _dac22, uint16_t _dac31, uint16_t _dac32, uint16_t _dac41, uint16_t _dac42)
{
	LDAC_1();
	
	DAC8562_WriteCmd_Parallel_4((3 << 19) | (0 << 16) | (_dac11 << 0), 
				(3 << 19) | (0 << 16) | (_dac21 << 0),
				(3 << 19) | (0 << 16) | (_dac31 << 0),
				(3 << 19) | (0 << 16) | (_dac41 << 0));
	
	DAC8562_WriteCmd_Parallel_4((3 << 19) | (1 << 16) | (_dac12 << 0), 
				(3 << 19) | (1 << 16) | (_dac22 << 0),
				(3 << 19) | (1 << 16) | (_dac32 << 0),
				(3 << 19) | (1 << 16) | (_dac42 << 0));
	
	LDAC_0();
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
