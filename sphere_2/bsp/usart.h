/* �����ֹ�ݹ���� ----------------------------------------------------------*/
#ifndef _USART_H
#define _USART_H

/* ������ͷ�ļ� --------------------------------------------------------------*/
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
/* �������� ------------------------------------------------------------------*/
void USART_Initializes(void);
void USART1_SendByte(uint8_t Data);
void USART1_SendNByte(uint8_t *pData, uint16_t Length);
void USART1_SendString(uint8_t *String);
char* USART1_GetLine(void);

#define printx(str) USART1_SendString((uint8_t*)str)
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

//!!VENTUSFF: �ǳ���Ҫ�����û����仰��������ͻ�ʧЧ
extern USART_GetLine g_tUSART_1_GetLine;

#endif /* _USART_H */

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
