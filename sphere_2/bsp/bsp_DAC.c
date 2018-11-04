/*
*********************************************************************************************************
*
*	ģ������ : DAC8562/8563 ����ģ��(��ͨ����16λDAC)
*	�ļ����� : bsp_dac8562.c
*	��    �� : V1.0
*	˵    �� : DAC8562/8563ģ���CPU֮�����SPI�ӿڡ�����������֧��Ӳ��SPI�ӿں����SPI�ӿڡ�
*			  ͨ�����л���
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2014-01-17  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp_DAC.h"

#define SOFT_SPI		/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */
//#define HARD_SPI		/* ������б�ʾʹ��CPU��Ӳ��SPI�ӿ� */

/*
	DAC8501ģ�����ֱ�Ӳ嵽STM32-V5������CN19��ĸ(2*4P 2.54mm)�ӿ���

    DAC8562/8563ģ��    STM32F407������
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
	DAC8562��������:
	1������2.7 - 5V;  ������ʹ��3.3V��
	4���ο���ѹ2.5V��ʹ���ڲ��ο�

	��SPI��ʱ���ٶ�Ҫ��: �ߴ�50MHz�� �ٶȺܿ�.
	SCLK�½��ض�ȡ����, ÿ�δ���24bit���ݣ� ��λ�ȴ�
*/

#if !defined(SOFT_SPI) && !defined(HARD_SPI)
 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
#endif

#ifdef SOFT_SPI		/* ���SPI */
	/* ����GPIO�˿� */
	#define RCC_SCLK 	RCC_AHB1Periph_GPIOF
	#define PORT_SCLK	GPIOF
	#define PIN_SCLK	GPIO_Pin_8
	
	#define RCC_DIN 	RCC_AHB1Periph_GPIOC
	#define PORT_DIN 	GPIOC
	#define PIN_DIN 	GPIO_Pin_4
	#define PIN_DIN2 	GPIO_Pin_5
	#define PIN_DIN3 	GPIO_Pin_8
	#define PIN_DIN4 	GPIO_Pin_9
	
	/* Ƭѡ */
	#define RCC_SYNC 	RCC_AHB1Periph_GPIOF
	#define PORT_SYNC	GPIOF
	#define PIN_SYNC	GPIO_Pin_7

	/* LDAC, ���Բ��� */
	#define RCC_LDAC	RCC_AHB1Periph_GPIOF
	#define PORT_LDAC	GPIOF
	#define PIN_LDAC	GPIO_Pin_6
	
	/* CLR, ���Բ��� */
	#define RCC_CLR 	RCC_AHB1Periph_GPIOD
	#define PORT_CLR	GPIOD
	#define PIN_CLR 	GPIO_Pin_6

	/* ���������0����1�ĺ� */
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

#ifdef HARD_SPI		/* Ӳ��SPI (δ��) */
	;
#endif

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitDAC8562
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� DAC8562
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitDAC8562(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef SOFT_SPI
	SYNC_1();	/* SYNC = 1 */

	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_SCLK | RCC_DIN | RCC_CLR, ENABLE);

	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

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
	
	/* LDAC pin inactive for DAC-B and DAC-A  ��ʹ��LDAC���Ÿ������� */
	//DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));
	//DAC8562_WriteCmd_Parallel_4_Same((6 << 19) | (3 << 16) | (3 << 0));
	
	/* LDAC pin active for DAC-B and DAC-A  ʹ��LDAC���Ÿ������� */
	DAC8562_WriteCmd_Parallel_4_Same((6 << 19) | (0 << 16) | (0 << 0));
	LDAC_1();	//Ӳ��LDACģʽ�£���ҪLDAC��ʼ����
	

	/* ��λ2��DAC���м�ֵ, ���2.5V */
	
	//DAC8562_SetData(0, 32767);
	//DAC8562_SetData(1, 32767);
	DAC8563_SetData_Simultaneously(32767,32767,32767,32767,32767,32767,32767,32767);
	//__NOP();//������������LDAC����֮�䣬��Ҫ5ns,�����ƺ�����Ҫ��
	LDAC_0();	//Ӳ��LDACģʽ�£�д��������ҪLDAC������ͬ������

	/* ѡ���ڲ��ο�����λ2��DAC������=2 ����λʱ���ڲ��ο��ǽ�ֹ��) */
	DAC8562_WriteCmd_Parallel_4_Same((7 << 19) | (0 << 16) | (1 << 0));
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_WriteCmd
*	����˵��: ��SPI���߷���24��bit���ݡ�
*	��    ��: _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DAC8562_WriteCmd(uint32_t _cmd)
{
	uint8_t i;
	SYNC_0();
	
	/*��DAC8562 SCLKʱ�Ӹߴ�50M����˿��Բ��ӳ� */
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
		//ʵ�ʼ����Է��֣�SCKL�������֮����³�����24ns
		SCLK_1();
		_cmd <<= 1;
		SCLK_0();
	}
	
	SYNC_1();
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_WriteCmd_Parallel_4
*	����˵��: ��4·DAC8563��SPI����ͬʱ����4·24��bit��ͬ�����ݡ�
*	��    ��: _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DAC8562_WriteCmd_Parallel_4(uint32_t _cmd1, uint32_t _cmd2, uint32_t _cmd3, uint32_t _cmd4)
{
	uint8_t i;
	SYNC_0();
	
	/*��DAC8562 SCLKʱ�Ӹߴ�50M����˿��Բ��ӳ� */
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
		//ʵ�ʼ����Է��֣�SCKL�������֮����³�����24ns
		SCLK_1();
		idx >>= 1;
		SCLK_0();
	}
	
	SYNC_1();
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_WriteCmd_Parallel_4
*	����˵��: ��4·DAC8563��SPI����ͬʱ����24bit��ͬ�����ݡ�
*	��    ��: _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DAC8562_WriteCmd_Parallel_4_Same(uint32_t _cmd)
{
	uint8_t i;
	uint16_t common = PIN_DIN | PIN_DIN2 | PIN_DIN3 | PIN_DIN4;
	SYNC_0();
	
	/*��DAC8562 SCLKʱ�Ӹߴ�50M����˿��Բ��ӳ� */
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
		//ʵ�ʼ����Է��֣�SCKL�������֮����³�����24ns
		SCLK_1();
		_cmd <<= 1;
		SCLK_0();
	}
	
	SYNC_1();
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_SetData
*	����˵��: ����DAC��������������¡�
*	��    ��: _ch, ͨ��, 0 , 1
*		     _data : ����
*	�� �� ֵ: ��
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
