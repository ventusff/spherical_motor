#include "bsp_Key.h"

extern _ctl_SYSTEM_CONTROL g_tSystemControl;

void bsp_Key_Initializes(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_START | RCC_STOP, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PE0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = PIN_START;
    GPIO_Init(PORT_START, &GPIO_InitStructure);

    /* Configure PE1 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = PIN_STOP;
    GPIO_Init(PORT_STOP, &GPIO_InitStructure);

    		/* Connect EXTA LineE to PI6 pin */
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

			/* Configure EXTA Line8 */
			EXTI_InitStructure.EXTI_Line = EXTI_Line0;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

			//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);

			/* Enable and set EXTI Line8 Interrupt to the lowest priority */
			NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);





            /* Connect EXTA LineE to PI6 pin */
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);

			/* Configure EXTA Line8 */
			EXTI_InitStructure.EXTI_Line = EXTI_Line1;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

			//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);

			/* Enable and set EXTI Line8 Interrupt to the lowest priority */
			NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

    
}

void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
        g_tSystemControl.STATUS = _ctl_STATUS_STOP;
		/* Clear the EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
        g_tSystemControl.STATUS = _ctl_STATUS_RUNNING;
		/* Clear the EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
