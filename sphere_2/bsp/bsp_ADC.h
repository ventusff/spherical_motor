#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f4xx.h"
#include "configuration.h"
/* ���������� */
/*
typedef enum
{
	AD_OS_NO = 0,
	AD_OS_X2 = 1,
	AD_OS_X4 = 2,
	AD_OS_X8 = 3,
	AD_OS_X16 = 4,
	AD_OS_X32 = 5,
	AD_OS_X64 = 6
}AD7606_OS_E;
*/

//3��ADC_port
typedef enum
{
	AD7606_PORT1 = 0,
	AD7606_PORT2 = 1,
	AD7606_PORT3 = 2
}AD7606_PORT;

/* AD���ݲɼ������� FIFO */
#define ADC_FIFO_SIZE	(8)	/* ���������� */

typedef struct
{
	uint8_t ucOS;			/* ���������ʣ�0 - 6. 0��ʾ�޹����� */
	uint8_t ucRange;		/* �������̣�0��ʾ����5V, 1��ʾ����10V */
	int16_t sNowAdc[8];		/* ��ǰADCֵ, �з����� */
  uint8_t ucADchannels;   /* ��ǰADC����ͨ���� */
}AD7606_VAR_T;

typedef struct
{
	/* FIFO �ṹ */
	uint16_t usRead;		/* ��ָ�� */
	uint16_t usWrite;		/* дָ�� */

	uint16_t usCount;		/* �����ݸ��� */
	uint8_t ucFull;			/* FIFO����־ */

	int16_t  sBuf[ADC_FIFO_SIZE][8];
}AD7606_FIFO_T;

typedef struct
{
	int16_t  RAWarray[3][8];
}AD7606_readings;

//3��ADC��A,B��ͨ����ǿ�Ʋ���
//�˴�����3��ADC:1,2,3�Ŀ�����������ŵ�ƽ�仯����

#define AD7606_CONV_1_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_6)
#define AD7606_CONV_1_H()		GPIO_SetBits(GPIOG, GPIO_Pin_6)

#define AD7606_CONV_2_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_7)
#define AD7606_CONV_2_H()		GPIO_SetBits(GPIOG, GPIO_Pin_7)

#define AD7606_CONV_3_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_8)
#define AD7606_CONV_3_H()		GPIO_SetBits(GPIOG, GPIO_Pin_8)

#define AD7606_RESET_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_11)
#define AD7606_RESET_H()		GPIO_SetBits(GPIOG, GPIO_Pin_11)

/* �����������̵�GPIO :  */
/*
#define AD7606_RANGE_H()	GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define AD7606_RANGE_L()	GPIO_ResetBits(GPIOA,GPIO_Pin_4)
*/

#define AD7606_1_READY()	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET)
#define AD7606_2_READY()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_RESET)
#define AD7606_3_READY()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_RESET)

/* AD7606 FSMC���ߵ�ַ��ֻ�ܶ�������д */
	//6			C			4			0			0			0			0			0
	//0110 	1100 	0100 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
	//A27,A26��FSMC_NE1,FSMC_NE2,FSMC_NE3,FSMC_NE4������ѡ4��bank
	//A25:0���Լ����ĵ�ַ

//���ε����ʹ��138��������3��AD7606����Ƭѡ
//���ӹ�ϵ��
	//G1���ߣ�G2A,G2B������FSMC_NE4(PG12)
	//A:A23,		B:A24,		C:A25;
	//Y0:AD7606_CS1,		Y1:AD7606_CS2,		Y2:AD7606_CS3
//ʹ�ܱ�
	//000:AD7606_1
	//001:AD7606_2
	//010:AD7606_3



//AD7606_1:   00 0000 x -> 00 0000 x
	//6			C			0			0			0			0			0			0
	//0110 	1100 	0000 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
#define AD7606_RESULT_1() *(__IO uint16_t *)0x6C000000

//AD7606_2:   01 0000 x -> 00 1000 x
	//6			D			0			0			0			0			0			0
	//0110 	1101 	0000 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
#define AD7606_RESULT_2() *(__IO uint16_t *)0x6D000000

//AD7606_3:   10 0000 x -> 01 0000 x
	//6			E			0			0			0			0			0			0
	//0110 	1110 	0000 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
#define AD7606_RESULT_3() *(__IO uint16_t *)0x6E000000

/* ---------------------  �������� -------------------- */
void bsp_InitAD7606(AD7606_VAR_T * _p_tAD7606,AD7606_readings * _p_tReadings);
//void AD7606_SetInputRange(uint8_t _ucRange);
void AD7606_Reset(void);
void AD7606_StartConvst(uint8_t ADC_port);
void AD7606_ReadNowAdc(uint8_t ADC_port);
#endif 

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
