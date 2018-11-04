#ifndef __BSP_MOTOR_DRIVE_H
#define __BSP_MOTOR_DRIVE_H
#include "stm32f4xx.h"
#include "bsp_ADC.h"
#include "bsp_DAC.h"
#include "usart.h"
#include "app_TimingCheck.h"
//ADC\DAC(motor_drive)工作时钟
#define motor_drive_clock_cmd	RCC_APB1Periph_TIM5
#define motor_drive_clock			TIM5
#define motor_drive_irqn	TIM5_IRQn

typedef struct {
    uint8_t AD7606_2_PREVIOUS;
    uint8_t AD7606_3_PREVIOUS;
    uint8_t FLAG_READY_1;
    uint8_t FLAG_READY_2;
    uint8_t FLAG_READY_3;
    uint8_t MainPID_Calculating;
}CoilDriveFLAGS;

/* 函数申明 ------------------------------------------------------------------*/
/* AD7606操作 -------------------------------------------------------------------*/
void AD7606_EnterAutoMode(uint32_t _ulFreq);//这是正式开始ADC的函数
void AD7606_ISR(uint8_t ADC_port);
uint8_t AD7606_HasNewData(uint8_t ADC_port);
uint8_t AD7606_FifoFull(uint8_t ADC_port);
uint8_t AD7606_ReadFifo(uint8_t ADC_port,int16_t *_usReadAdc_x8);//读取8个ADC值
void AD7606_StartRecord(uint8_t ADC_port,uint32_t _ulFreq);
void AD7606_StopRecord(void);
/* DAC8563操作 -------------------------------------------------------------------*/
//详见bsp_DAC.h
extern void DAC8563_SetData_Simultaneously(uint16_t _dac11, uint16_t _dac12,
         uint16_t _dac21, uint16_t _dac22, uint16_t _dac31, uint16_t _dac32, 
         uint16_t _dac41, uint16_t _dac42);

/* total -------------------------------------------------------------------*/
void bsp_motor_drive_initializes(void);
void MainClockISR(void);//主时钟的触发先决中断处理函数。该中断处理函数结束后，触发主时钟.
void _bsp_StartMainClock(void);
void _bsp_StopMainClock(void);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
