#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f4xx.h"
#include "configuration.h"
/* 过采样倍率 */
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

//3个ADC_port
typedef enum
{
	AD7606_PORT1 = 0,
	AD7606_PORT2 = 1,
	AD7606_PORT3 = 2
}AD7606_PORT;

/* AD数据采集缓冲区 FIFO */
#define ADC_FIFO_SIZE	(8)	/* 总体样本数 */

typedef struct
{
	uint8_t ucOS;			/* 过采样倍率，0 - 6. 0表示无过采样 */
	uint8_t ucRange;		/* 输入量程，0表示正负5V, 1表示正负10V */
	int16_t sNowAdc[8];		/* 当前ADC值, 有符号数 */
  uint8_t ucADchannels;   /* 当前ADC采样通道数 */
}AD7606_VAR_T;

typedef struct
{
	/* FIFO 结构 */
	uint16_t usRead;		/* 读指针 */
	uint16_t usWrite;		/* 写指针 */

	uint16_t usCount;		/* 新数据个数 */
	uint8_t ucFull;			/* FIFO满标志 */

	int16_t  sBuf[ADC_FIFO_SIZE][8];
}AD7606_FIFO_T;

typedef struct
{
	int16_t  RAWarray[3][8];
}AD7606_readings;

//3个ADC的A,B两通道已强制并联
//此处定义3个ADC:1,2,3的开关与控制引脚电平变化函数

#define AD7606_CONV_1_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_6)
#define AD7606_CONV_1_H()		GPIO_SetBits(GPIOG, GPIO_Pin_6)

#define AD7606_CONV_2_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_7)
#define AD7606_CONV_2_H()		GPIO_SetBits(GPIOG, GPIO_Pin_7)

#define AD7606_CONV_3_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_8)
#define AD7606_CONV_3_H()		GPIO_SetBits(GPIOG, GPIO_Pin_8)

#define AD7606_RESET_L()		GPIO_ResetBits(GPIOG, GPIO_Pin_11)
#define AD7606_RESET_H()		GPIO_SetBits(GPIOG, GPIO_Pin_11)

/* 设置输入量程的GPIO :  */
/*
#define AD7606_RANGE_H()	GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define AD7606_RANGE_L()	GPIO_ResetBits(GPIOA,GPIO_Pin_4)
*/

#define AD7606_1_READY()	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET)
#define AD7606_2_READY()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_RESET)
#define AD7606_3_READY()	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == Bit_RESET)

/* AD7606 FSMC总线地址，只能读，无需写 */
	//6			C			4			0			0			0			0			0
	//0110 	1100 	0100 	0000 	0000 	0000 	0000 	0000
	//31:28	27:24	23:20	19:16	15:12	11:8	7:4		3:0
	//A27,A26由FSMC_NE1,FSMC_NE2,FSMC_NE3,FSMC_NE4决定，选4个bank
	//A25:0是自己定的地址

//球形电机：使用138译码器对3个AD7606进行片选
//连接关系：
	//G1常高，G2A,G2B均连接FSMC_NE4(PG12)
	//A:A23,		B:A24,		C:A25;
	//Y0:AD7606_CS1,		Y1:AD7606_CS2,		Y2:AD7606_CS3
//使能表：
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

/* ---------------------  函数声明 -------------------- */
void bsp_InitAD7606(AD7606_VAR_T * _p_tAD7606,AD7606_readings * _p_tReadings);
//void AD7606_SetInputRange(uint8_t _ucRange);
void AD7606_Reset(void);
void AD7606_StartConvst(uint8_t ADC_port);
void AD7606_ReadNowAdc(uint8_t ADC_port);
#endif 

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
