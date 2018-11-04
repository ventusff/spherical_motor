/* 定义防止递归包含 ----------------------------------------------------------*/
#ifndef _USART_H
#define _USART_H

/* 包含的头文件 --------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "configuration.h"
#include <stdio.h>

typedef struct {
	uint8_t WaitingFlag;
	uint8_t ReadyFlag;
  uint16_t ReceiveCount;
  uint8_t receive_buffer[_cfg_USART_BUFFER_MAX];
  uint16_t strLen;
}USART_GetLine;
/* 函数申明 ------------------------------------------------------------------*/
void USART_Initializes(void);
void USART1_SendByte(uint8_t Data);
void USART1_SendNByte(uint8_t *pData, uint16_t Length);
void USART1_SendString(uint8_t *String);
char* USART1_GetLine(void);

#define printx(str) USART1_SendString((uint8_t*)str)
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

//!!VENTUSFF: 非常重要！如果没有这句话，作用域就会失效
extern USART_GetLine g_tUSART_1_GetLine;

#endif /* _USART_H */

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
