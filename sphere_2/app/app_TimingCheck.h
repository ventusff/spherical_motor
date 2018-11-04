#ifndef __APP_TIMING_CHECK__H
#define __APP_TIMING_CHECK__H

#include "stm32f4xx.h"
#include "configuration.h"
void _tim_TickMainTimer(void);
void _tim_TickSecondaryTimer(void);
void _tim_TickDebugTimer(void);
void _tim_TickGroundTruth(void);
void _tim_TickTest(void);

uint32_t _tim_GetDebugTimer(void);
uint32_t _tim_GetMainTimer(void);
uint32_t _tim_GetSecondaryTimer(void);
uint32_t _tim_GetGroundTruth(void);
uint32_t _tim_GetTestTimer(void);
void _tim_ZeroAllTimer(void);

void _tim_StartTest(void);
void _tim_StopTest(void);
uint8_t _tim_IsTesting(void);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
