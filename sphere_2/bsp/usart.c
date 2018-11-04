/* 包含的头文件 --------------------------------------------------------------*/
#include "usart.h"
#include "stm32f4xx_it.h"
USART_GetLine g_tUSART_1_GetLine;
USART_GetLine g_tUSART_2_GetLine;

/************************************************
函数名称 ： USART_GPIO_Configuration
功    能 ： USART所使用管脚输出输入定义
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void USART_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* RCC时钟配置 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  /* 定义USART-TX 引脚为复用输出 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                          //IO口的第9脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO口速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                       //IO口复用
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART1输出IO口

  /* 定义 USART-Rx 引脚为悬空输入 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                         //IO口的第10脚
  GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART1输入IO口

  //归零装置接收
  {
    /* RCC时钟配置 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* 定义USART-TX 引脚为复用输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                          //IO口的第9脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO口速度
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                       //IO口复用
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART2输出IO口

    /* 定义 USART-Rx 引脚为悬空输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                         //IO口的第10脚
    GPIO_Init(GPIOA, &GPIO_InitStructure);                             //USART2输入IO口
  }
}

/************************************************
函数名称 ： USART_Configuration
功    能 ： 配置USART
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  /******************************************************************
   USART参数初始化:  波特率     传输位数   停止位数  校验位数
                     115200        8          1       0(NO)
  *******************************************************************/
  USART_InitStructure.USART_BaudRate = 115200;                       //设定传输速率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;        //设定传输数据位数
  USART_InitStructure.USART_StopBits = USART_StopBits_1;             //设定停止位个数
  USART_InitStructure.USART_Parity = USART_Parity_No ;               //不用校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不用流量控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //使用接收和发送功能
  USART_Init(USART1, &USART_InitStructure);                          //初始化USART1

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                     //使能USART1接收中断

  USART_Cmd(USART1, ENABLE);                                         //使能USART1
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
  /* 外设中断1 --- 电脑串口 */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  //IRQ通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          //主优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                 //从优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                    //使能通道
  NVIC_Init(&NVIC_InitStructure);

  /******************************************************************
   USART参数初始化:  波特率     传输位数   停止位数  校验位数
                     115200        8          1       0(NO)
  *******************************************************************/
  USART_InitStructure.USART_BaudRate = 115200;                       //设定传输速率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;        //设定传输数据位数
  USART_InitStructure.USART_StopBits = USART_StopBits_1;             //设定停止位个数
  USART_InitStructure.USART_Parity = USART_Parity_No ;               //不用校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不用流量控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //使用接收和发送功能
  USART_Init(USART2, &USART_InitStructure);                          //初始化USART2

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                     //使能USART2接收中断

  USART_Cmd(USART2, ENABLE);                                         //使能USART2
	
	
  /* 外设中断2 --- 归零设备接收串口 */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                  //IRQ通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          //主优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                 //从优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                    //使能通道
  NVIC_Init(&NVIC_InitStructure);
}

/************************************************
函数名称 ： USART_Initializes
功    能 ： 串口初始化
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
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
函数名称 ： USART1_SendChar
功    能 ： 串口1发送一个字符
参    数 ： Data --- 数据
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void USART1_SendByte(uint8_t Data)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);       //等待发送寄存器为空
  USART_SendData(USART1, Data);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);        //等待发送完成
}

/************************************************
函数名称 ： USART1_SendNByte
功    能 ： 串口1发送N个字符
参    数 ： pData ---- 字符串
            Length --- 长度
返 回 值 ： 无
作    者 ： strongerHuang
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
函数名称 ： USART1_SendString
功    能 ： 串口1发送一串字符
参    数 ： string --- 字符串
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void USART1_SendString(uint8_t *String)
{
  while((*String) != '\0')
  {
    USART1_SendByte(*String);
    String++;
  }
  USART1_SendByte(0x0A);                         //换行
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
		if(g_tUSART_1_GetLine.WaitingFlag && g_tUSART_1_GetLine.ReadyFlag == 0)
		{
			uint16_t data = USART_ReceiveData(USART1);  //接收到的字符数据
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
    //USART1_SendByte(USART_ReceiveData(USART1));  //发送接收到的字符数据
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
			uint16_t data = USART_ReceiveData(USART2);  //接收到的字符数据
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

        //发送接收到的数据
        //printx(g_tUSART_2_GetLine.receive_buffer);
      }
      else
      {
        g_tUSART_2_GetLine.ReceiveCount++;
      }
  }
}




/******************************************* 串口重定义 *******************************************/
/************************************************
函数名称 ： fputc
功    能 ： 重定义putc函数
参    数 ： ch --- 字符
            *f --- 文件指针
返 回 值 ： 字符
作    者 ： strongerHuang
*************************************************/
int fputc(int ch, FILE *f)
{
  USART1_SendByte((uint8_t)ch);

  return ch;
}

/************************************************
函数名称 ： fgetc
功    能 ： 重定义getc函数
参    数 ： *f --- 文件指针
返 回 值 ： 字符
作    者 ： strongerHuang
*************************************************/
int fgetc(FILE *f)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

  return (int)USART_ReceiveData(USART1);
}


/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
