#include "app_TimingCheck.h"

uint32_t DebugTimerCheck = 0;
uint32_t MainTimerCheck = 0;
uint32_t SecondaryTimerCheck = 0;
uint32_t TimerGroundTruth = 0;
uint32_t TestTimerCheck = 0;
uint8_t  Testing = 0;

void _tim_TickMainTimer(void)
{
	static uint32_t counter = 0;
	counter++;
	if(counter == _tim_MAIN_CHECK_COUNT)
	{
		counter = 0;
		MainTimerCheck++;
	}
}

void _tim_TickSecondaryTimer(void)
{
    static uint16_t counter = 0;
    counter++;
    if(counter == _tim_SECONDARY_CHECK_COUNT)
    {
        counter = 0;
        SecondaryTimerCheck++;
    }
}

void _tim_TickDebugTimer(void)
{
	DebugTimerCheck++;
}

void _tim_TickGroundTruth(void)
{
    TimerGroundTruth++;
}

uint32_t _tim_GetDebugTimer(void)
{
    return DebugTimerCheck;
}

uint32_t _tim_GetMainTimer(void)
{   
    return MainTimerCheck;
}

uint32_t _tim_GetSecondaryTimer(void)
{
    return SecondaryTimerCheck;
}

uint32_t _tim_GetGroundTruth(void)
{
    return TimerGroundTruth;
}

uint32_t _tim_GetTestTimer(void)
{
    return TestTimerCheck;
}

void _tim_ZeroAllTimer(void)
{
    MainTimerCheck = 0;
    DebugTimerCheck = 0;
    SecondaryTimerCheck = 0;
    TimerGroundTruth = 0;
}

void _tim_TickTest(void)
{
	if(Testing)
		TestTimerCheck++;
}

void _tim_StartTest(void)
{
	TestTimerCheck = 0;
	Testing = 1;
}

void _tim_StopTest(void)
{
	Testing = 0;
}

uint8_t _tim_IsTesting(void)
{
	return Testing;
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
