
/* 定义防止递归包含 ----------------------------------------------------------*/
#ifndef _BSP_H
#define _BSP_H

/* 包含的头文件 --------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "bsp_motor_drive.h"
#include "bsp_ADNS9800.h"
#include "bsp_Key.h"
#include "usart.h"
#include "timer.h"


/* 宏定义 --------------------------------------------------------------------*/
#define PORT_LED                  GPIOD
#define PIN_LED                   GPIO_Pin_15

/* LED开关 */
#define LED_ON                    GPIO_SetBits(PORT_LED, PIN_LED)
#define LED_OFF                   GPIO_ResetBits(PORT_LED, PIN_LED)
#define LED_TOGGLE                GPIO_ToggleBits(PORT_LED, PIN_LED)


/* 函数申明 ------------------------------------------------------------------*/
void BSP_Initializes(void);

#endif /* _BSP_H */

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
