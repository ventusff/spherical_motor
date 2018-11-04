#ifndef __BSP_KEY__H
#define __BSP_KEY__H
#endif

#include "configuration.h"

#define PORT_START  GPIOE
#define PIN_START   GPIO_Pin_0
#define RCC_START   RCC_AHB1Periph_GPIOE

#define PORT_STOP   GPIOE
#define PIN_STOP    GPIO_Pin_1
#define RCC_STOP    RCC_AHB1Periph_GPIOE

void bsp_Key_Initializes(void);

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
