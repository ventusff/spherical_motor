/**
 * �ṩADC,DAC�Ķ�ȡ��д���������ӿ�
 * 
 * */
#include "bsp_motor_drive.h"

extern void MainClockTask(void);


AD7606_VAR_T g_tAD7606;		/* ����1��ȫ�ֱ���������һЩ���� */
AD7606_FIFO_T g_tAdcFifo[3];	/* ����FIFO�ṹ����� */
AD7606_readings g_tReadings;
volatile CoilDriveFLAGS g_tCoilDriveFLAGS;
//volatile indicates that a variable can be changed by a backgroud routine.
//ֻ�м�����仰��main() �� g_tCoilDriveFLAGS���޸Ĳ�����Ч.

/*
*********************************************************************************************************
*		����ĺ������ڶ�ʱ�ɼ�ģʽ�� TIM5Ӳ����ʱ�ж��ж�ȡADC���������ȫ��FIFO
*
*
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	�� �� ��: AD7606_EnterAutoMode
*	����˵��: ����Ӳ���������Զ��ɼ�ģʽ������洢��FIFO��������
*	��    �Σ�_ulFreq : ����Ƶ�ʣ���λHz��	1k��2k��5k��10k��20K��50k��100k��200k
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/*
	���վ�����ֻ��AD7606_1ʹ���жϷ�ʽ��
	������ʹ�ö�ʱɨ�跽ʽ��
	����Ҫ��ȡƵ��Ϊ1000Hz���������ɨ��Ƶ��Ϊ2000Hz��
	���߶�����AD7606_1�Ķ�ʱ�жϣ�ֻ�����з�Ƶ��
	��Ҫ�Ƿ��֣����2��AD7606����Ϊͬһ���жϺ����������ڶ�ʹ��EXTI_Line_9_5�жϣ��жϻᱻ�̵���
*/
/*
	������Գ����޸����ذ��·����֤3��BUSY���ž����������жϵ��ж�ͨ��
*/
void AD7606_EnterAutoMode(uint32_t _ulFreq)
{
    g_tAD7606.ucADchannels = 8;
    bsp_InitAD7606(&g_tAD7606,&g_tReadings);

    //����TIM5ΪADC�Զ�ʱ��
	{
		uint32_t uiTIMxCLK;
		uint16_t usPrescaler;
		uint16_t usPeriod;

		//TIM_DeInit(TIM5);	/* ��λTIM��ʱ�� */

	    /*-----------------------------------------------------------------------
			system_stm32f4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�

			HCLK = SYSCLK / 1     (AHB1Periph)
			PCLK2 = HCLK / 2      (APB2Periph)
			PCLK1 = HCLK / 4      (APB1Periph)

			��ΪAPB1 prescaler != 1, ���� APB1�ϵ�TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
			��ΪAPB2 prescaler != 1, ���� APB2�ϵ�TIMxCLK = PCLK2 x 2 = SystemCoreClock;

			APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
			APB2 ��ʱ���� TIM1, TIM8 ,TIM9, TIM10, TIM11
		*/

		uiTIMxCLK = SystemCoreClock / 2;//168000000/2

		if (_ulFreq < 3000)
		{
			usPrescaler = 100 - 1;					/* ��Ƶ�� = 10 */
			usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* �Զ���װ��ֵ */
		}
		else	/* ����4K��Ƶ�ʣ������Ƶ */
		{
			usPrescaler = 0;					/* ��Ƶ�� = 1 */
			usPeriod = uiTIMxCLK / _ulFreq - 1;	/* �Զ���װ��ֵ */
		}


        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        RCC_APB1PeriphClockCmd(motor_drive_clock_cmd, ENABLE);//ʹ��TIM2����TIM2��ʱ��

        TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;        //Ԥ��Ƶֵ
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //���ϼ���ģʽ
        TIM_TimeBaseStructure.TIM_Period = usPeriod;             //��ʱ����
        //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //ʱ�ӷ�Ƶ���ӡ���������ʱ���޴˹���
        TIM_TimeBaseInit(motor_drive_clock, &TIM_TimeBaseStructure);
        
    }

    //����TIM5���ж�Ϊ��ʼһ��
  {
				NVIC_InitTypeDef NVIC_InitStructure;
        TIM_ClearFlag(motor_drive_clock, TIM_FLAG_Update); 
        /* ��������ж� */
        TIM_ITConfig(motor_drive_clock,TIM_IT_Update,ENABLE );
        
        /* ʹ��Ԥ��Ƶֵ */
        TIM_ARRPreloadConfig(motor_drive_clock, ENABLE);                                //ʹ������ֵ
        
        /* NVIC �ж����ȼ����� */
        NVIC_InitStructure.NVIC_IRQChannel = motor_drive_irqn; 
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
        NVIC_Init(&NVIC_InitStructure); 
        
	}

	/* ����PA8, BUSY ��Ϊ�ж�����ڣ��½��ش��� */
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
*	�� �� ��: AD7606_ISR
*	����˵��: ��ʱ�ɼ��жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_ISR(uint8_t ADC_port)
{
	uint8_t i;

	AD7606_ReadNowAdc(ADC_port);

	//�����д��FIFO��������usWrite,Countֵ
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
		g_tAdcFifo[ADC_port].ucFull = 1;		/* FIFO ������������������������ */
	}

}

void MainClockISR(void)
{
	static uint16_t COUNT_DIVIDE = 1;
	if(_cfg_USE_ADC_1)
	{
		AD7606_StartConvst(AD7606_PORT1);
	}
		
	


	//ֻ��ÿ��100��Ƶ���ʼ��ʼת��
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
		//���AD7606_2,AD7606_3��ready״̬
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

//TIM5��ʱ�жϣ���ʼAD7606��ת��
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(motor_drive_clock, TIM_IT_Update) != RESET) 
	{
		MainClockISR();
		TIM_ClearITPendingBit(motor_drive_clock, TIM_IT_Update ); 
	}
	
}

/*
����һЩ���β�����AD7606_2ʹ��PB8, AD7606_1ʹ��PA8
���ǲ���ͬʱ�����������жϡ�
��������ֻ�ܸ�AD7606_2����ʹ��һ�׶�ʱɨ������ȡ�ķ�ʽ��
AD7606_3���Լ������ö�ʱ��ȡ�ķ�ʽ��
AD7606_2Ҳ����������ö�ʱ��ȡ�ķ�ʽ����Ҫ��Ϊ�˱�֤ʱ��һֱ��Ч
*/
/*
*********************************************************************************************************
*	�� �� ��: EXTI9_5_IRQHandler
*	����˵��: �ⲿ�жϷ��������ڡ�PH12/AD7606_BUSY �½����жϴ���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		AD7606_ISR(AD7606_PORT1);
		g_tCoilDriveFLAGS.FLAG_READY_1 = 1;
		//_tim_TickMainTimer();
		
		//VENTUSFF: ֱ�ӷ����ж��У�
		//���ǵ�Ŀǰ20khz��50usһ�����ڣ��㹻���20us��DACд��
		//���ǵ��������Ҫ������
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
*	�� �� ��: AD7606_HasNewData
*	����˵��: �ж�FIFO���Ƿ���������
*	��    �Σ�_usReadAdc : ���ADC����ı���ָ��
*	�� �� ֵ: 1 ��ʾ�У�0��ʾ��������
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
*	�� �� ��: AD7606_FifoFull
*	����˵��: �ж�FIFO�Ƿ���
*	��    �Σ�_usReadAdc : ���ADC����ı���ָ��
*	�� �� ֵ: 1 ��ʾ����0��ʾδ��
*********************************************************************************************************
*/
uint8_t AD7606_FifoFull(uint8_t ADC_port)
{
	return g_tAdcFifo[ADC_port].ucFull;
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_ReadFifo
*	����˵��: ��FIFO�ж�ȡһ��ADCֵ
*	��    �Σ�_usReadAdc_x8 : ���ADC����ı���ָ��,8��������
*	�� �� ֵ: 1 ��ʾOK��0��ʾ��������
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
		#define ENABLE_INT()	__set_PRIMASK(0)	// ʹ��ȫ���ж� 
		#define DISABLE_INT()	__set_PRIMASK(1)	// ��ֹȫ���ж� 
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
*	�� �� ��: AD7606_StartRecord
*	����˵��: ��ʼ�ɼ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_StartRecord(uint8_t ADC_port, uint32_t _ulFreq)
{
	AD7606_StopRecord();

	AD7606_Reset();					/* ��λӲ�� */
	AD7606_StartConvst(ADC_port);			/* ���������������1������ȫ0������ */
	
	g_tAdcFifo[ADC_port].usRead = 0;			/* �����ڿ���TIM2֮ǰ��0 */
	g_tAdcFifo[ADC_port].usWrite = 0;
	g_tAdcFifo[ADC_port].usCount = 0;
	g_tAdcFifo[ADC_port].ucFull = 0;

	AD7606_EnterAutoMode(_ulFreq);
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_StopRecord
*	����˵��: ֹͣ�ɼ���ʱ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_StopRecord(void)
{
	TIM_Cmd(TIM5, DISABLE);

    //�������ר���ڽ�PH12ר����PWM���ʱ����ʵֻ����ΪPWM�����ǡ�ú�ԭ�������CONVST�ڳ�ͻ��
	/* ��PH12 ��������Ϊ��ͨ����� */
    /*
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		// ʹ�� GPIOʱ�� 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOH, &GPIO_InitStructure);
	}
    */

	AD7606_CONV_1_H();					/* ����ת����GPIOƽʱ����Ϊ�� */	\
  AD7606_CONV_2_H();
	AD7606_CONV_3_H();
}

void _bsp_StartMainClock(void)
{
	/* ʹ��motor_drive_clock */
    TIM_Cmd(motor_drive_clock, ENABLE);
}

void _bsp_StopMainClock(void)
{
	/* ʹ��motor_drive_clock */
    TIM_Cmd(motor_drive_clock, DISABLE);
}

/******************************  DAC8563 ��ص�����   *********************************/

/******************************  �ܹ�����   *********************************/
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
