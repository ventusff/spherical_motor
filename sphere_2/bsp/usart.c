/* ������ͷ�ļ� --------------------------------------------------------------*/
#include "usart.h"
#include "stm32f4xx_it.h"
USART_GetLine g_tUSART_1_GetLine;
USART_GetLine g_tUSART_2_GetLine;

/************************************************
�������� �� USART_GPIO_Configuration
��    �� �� USART��ʹ�ùܽ�������붨��
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* RCCʱ������ */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  /* ����USART-TX ����Ϊ������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                          //IO�ڵĵ�9��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO���ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                       //IO�ڸ���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART1���IO��

  /* ���� USART-Rx ����Ϊ�������� */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                         //IO�ڵĵ�10��
  GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART1����IO��

  //����װ�ý���
  {
    /* RCCʱ������ */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* ����USART-TX ����Ϊ������� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                          //IO�ڵĵ�9��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO���ٶ�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                       //IO�ڸ���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART2���IO��

    /* ���� USART-Rx ����Ϊ�������� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                         //IO�ڵĵ�10��
    GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART2����IO��
  }
}

/************************************************
�������� �� USART_Configuration
��    �� �� ����USART
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  /******************************************************************
   USART������ʼ��:  ������     ����λ��   ֹͣλ��  У��λ��
                     115200        8          1       0(NO)
  *******************************************************************/
  USART_InitStructure.USART_BaudRate = 115200;                       //�趨��������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;        //�趨��������λ��
  USART_InitStructure.USART_StopBits = USART_StopBits_1;             //�趨ֹͣλ����
  USART_InitStructure.USART_Parity = USART_Parity_No ;               //����У��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //ʹ�ý��պͷ��͹���
  USART_Init(USART1, &USART_InitStructure);                          //��ʼ��USART1

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                     //ʹ��USART1�����ж�

  USART_Cmd(USART1, ENABLE);                                         //ʹ��USART1
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
  /* �����ж�1 --- ���Դ��� */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  //IRQͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                 //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                    //ʹ��ͨ��
  NVIC_Init(&NVIC_InitStructure);

  /******************************************************************
   USART������ʼ��:  ������     ����λ��   ֹͣλ��  У��λ��
                     115200        8          1       0(NO)
  *******************************************************************/
  USART_InitStructure.USART_BaudRate = 115200;                       //�趨��������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;        //�趨��������λ��
  USART_InitStructure.USART_StopBits = USART_StopBits_1;             //�趨ֹͣλ����
  USART_InitStructure.USART_Parity = USART_Parity_No ;               //����У��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //ʹ�ý��պͷ��͹���
  USART_Init(USART2, &USART_InitStructure);                          //��ʼ��USART2

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                     //ʹ��USART2�����ж�

  USART_Cmd(USART2, ENABLE);                                         //ʹ��USART2
	
	
  /* �����ж�2 --- �����豸���մ��� */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                  //IRQͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                 //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                    //ʹ��ͨ��
  NVIC_Init(&NVIC_InitStructure);
}

/************************************************
�������� �� USART_Initializes
��    �� �� ���ڳ�ʼ��
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART_Initializes(void)
{
  USART_GPIO_Configuration();
  USART_Configuration();
  g_tUSART_1_GetLine.ReadyFlag = 0;
  g_tUSART_1_GetLine.WaitingFlag = 0;
  g_tUSART_1_GetLine.ReceiveCount = 0;
  g_tUSART_1_GetLine.strLen = 0;
}

/************************************************
�������� �� USART1_SendChar
��    �� �� ����1����һ���ַ�
��    �� �� Data --- ����
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART1_SendByte(uint8_t Data)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);       //�ȴ����ͼĴ���Ϊ��
  USART_SendData(USART1, Data);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);        //�ȴ��������
}

/************************************************
�������� �� USART1_SendNByte
��    �� �� ����1����N���ַ�
��    �� �� pData ---- �ַ���
            Length --- ����
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART1_SendNByte(uint8_t *pData, uint16_t Length)
{
  while(Length--)
  {
    USART1_SendByte(*pData);
    pData++;
  }
}

/************************************************
�������� �� USART1_SendString
��    �� �� ����1����һ���ַ�
��    �� �� string --- �ַ���
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART1_SendString(uint8_t *String)
{
  while((*String) != '\0')
  {
    USART1_SendByte(*String);
    String++;
  }
  USART1_SendByte(0x0A);                         //����
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
		if(g_tUSART_1_GetLine.WaitingFlag && g_tUSART_1_GetLine.ReadyFlag == 0)
		{
			uint16_t data = USART_ReceiveData(USART1);  //���յ����ַ�����
      g_tUSART_1_GetLine.receive_buffer[g_tUSART_1_GetLine.ReceiveCount] = (uint8_t)data;
      if( (
						data == _cfg_USART_RECEIVE_END_FLAG_2 
						&& g_tUSART_1_GetLine.receive_buffer[g_tUSART_1_GetLine.ReceiveCount - 1] == _cfg_USART_RECEIVE_END_FLAG 
						&& g_tUSART_1_GetLine.ReceiveCount > 0
					) 
					|| g_tUSART_1_GetLine.ReceiveCount == _cfg_USART_BUFFER_MAX - 2)
      {
        g_tUSART_1_GetLine.receive_buffer[g_tUSART_1_GetLine.ReceiveCount + 1] = '\0';
        g_tUSART_1_GetLine.strLen = g_tUSART_1_GetLine.ReceiveCount + 1;
        g_tUSART_1_GetLine.ReceiveCount = 0;
        g_tUSART_1_GetLine.ReadyFlag = 1;
      }
      else
      {
        g_tUSART_1_GetLine.ReceiveCount++;
      }
		}
    //USART1_SendByte(USART_ReceiveData(USART1));  //���ͽ��յ����ַ�����
  }
}

char* USART1_GetLine(void)
{
  g_tUSART_1_GetLine.WaitingFlag = 1;
  while(!g_tUSART_1_GetLine.ReadyFlag)
  {
    ;
  }
  g_tUSART_1_GetLine.WaitingFlag = 0;
  g_tUSART_1_GetLine.ReadyFlag = 0;
	return (char*) g_tUSART_1_GetLine.receive_buffer;
}

void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
			uint16_t data = USART_ReceiveData(USART2);  //���յ����ַ�����
      g_tUSART_2_GetLine.receive_buffer[g_tUSART_2_GetLine.ReceiveCount] = (uint8_t)data;
      if( (
						data == _cfg_USART_RECEIVE_END_FLAG_2 
						&& g_tUSART_2_GetLine.receive_buffer[g_tUSART_2_GetLine.ReceiveCount - 1] == _cfg_USART_RECEIVE_END_FLAG 
						&& g_tUSART_2_GetLine.ReceiveCount > 0
					) 
					|| g_tUSART_2_GetLine.ReceiveCount == _cfg_USART_BUFFER_MAX - 2)
      {
        g_tUSART_2_GetLine.receive_buffer[g_tUSART_2_GetLine.ReceiveCount + 1] = '\0';
        g_tUSART_2_GetLine.strLen = g_tUSART_2_GetLine.ReceiveCount + 1;
        g_tUSART_2_GetLine.ReceiveCount = 0;
        g_tUSART_2_GetLine.ReadyFlag = 1;

        //���ͽ��յ�������
        //printx(g_tUSART_2_GetLine.receive_buffer);
      }
      else
      {
        g_tUSART_2_GetLine.ReceiveCount++;
      }
  }
}




/******************************************* �����ض��� *******************************************/
/************************************************
�������� �� fputc
��    �� �� �ض���putc����
��    �� �� ch --- �ַ�
            *f --- �ļ�ָ��
�� �� ֵ �� �ַ�
��    �� �� strongerHuang
*************************************************/
int fputc(int ch, FILE *f)
{
  USART1_SendByte((uint8_t)ch);

  return ch;
}

/************************************************
�������� �� fgetc
��    �� �� �ض���getc����
��    �� �� *f --- �ļ�ָ��
�� �� ֵ �� �ַ�
��    �� �� strongerHuang
*************************************************/
int fgetc(FILE *f)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

  return (int)USART_ReceiveData(USART1);
}


/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
