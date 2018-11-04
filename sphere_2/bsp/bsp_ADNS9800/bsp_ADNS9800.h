#ifndef __BSP_ADNS9800_H
#define __BSP_ADNS9800_H

#include "stm32f4xx.h"
#include "A9800Chip.h"
#include "A9800Srom.h"
#include "usart.h"
#include "timer.h"
#include "configuration.h"

/***************************************************************************************************
����:   ϵͳ����.
***************************************************************************************************/
#define SYS_CLK_FREQ        (72000000UL)        //ϵͳʱ��.
#define SOFT_VER_STR        "MOUSE 20150728"    //MCU����汾��.

typedef enum    //������.
{
    OP_FAIL,
    OP_SUCCESS = !OP_FAIL
}OP;

typedef enum
{
	ADNS9800_CHIP1 = 0,
	ADNS9800_CHIP2 = 1
}ADNS9800_CHIP;

typedef struct{
    uint8_t   g_uA9800Cpi;     //CPIֵ.
    uint8_t   g_uA9800Lift;                  //̧��߶�����.

    int16_t MotionX_1;
    int16_t MotionY_1;
    int16_t MotionX_2;
    int16_t MotionY_2;

    int32_t  PosX_1;                //X��λ��.
    int32_t  PosY_1;                //Y��λ��.

    int32_t  PosX_2;              //X��λ��.��2�����оƬ
    int32_t  PosY_2;              //Y��λ��.��2�����оƬ

    uint32_t  TotalMotionX;						//x�����ֵλ�ƣ����оƬ-1
    uint32_t  TotalMotionY;						//y�����ֵλ�ƣ����оƬ-1

    uint32_t  TotalMotionX_2;					//x�����ֵλ�ƣ����оƬ-2
    uint32_t  TotalMotionY_2;					//y�����ֵλ�ƣ����оƬ-2
} ADNS9800_VAR_T;

/***************************************************************************************************
����:   ���з���.
***************************************************************************************************/
//����.
extern void A9800SetCpi(uint8_t uCpi);
extern void A9800SetLift(uint8_t uLift);
extern uint8_t    SpiXRead(uint8_t chip);                 //��.��֮ǰ���뱣֤Ƭѡ��Ч.
extern void     SpiXWrite(uint8_t uWriteData, uint8_t chip);    //д.д֮ǰ���뱣֤Ƭѡ��Ч.

//����.
extern void A9800OnTimerTask(void);

//״̬.
extern void A9800XSInit(uint8_t bIap);
extern void SpiXSInit(void);                 //��ʼ��.
void bsp_ADNS9800_Initializes(void);
/**************************************************************************************************/
#endif //#ifndef __A9800_H__

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
