#ifndef __BSP_DAC_H
#define __BSP_DAC_H

#include "stm32f4xx.h"
#include "configuration.h"

void bsp_InitDAC8562(void);
void DAC8562_SetData(uint8_t _ch, uint16_t _dac);
void DAC8562_WriteCmd(uint32_t _cmd);

//并行需要
void DAC8563_SetData_Simultaneously(uint16_t _dac11, uint16_t _dac12, uint16_t _dac21, uint16_t _dac22, uint16_t _dac31, uint16_t _dac32, uint16_t _dac41, uint16_t _dac42);
void DAC8562_WriteCmd_Parallel_4(uint32_t _cmd1, uint32_t _cmd2, uint32_t _cmd3, uint32_t _cmd4);
void DAC8562_WriteCmd_Parallel_4_Same(uint32_t _cmd);
void DAC8563_ZeroData_Simultaneously(void);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
