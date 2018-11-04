#include "bsp.h"
#include "timer.h"
#include "usart.h"
#include "configuration.h"
#include "app_Rotor_Rotate.h"
#include "app_Sub_PID_current.h"
#include "app_Main_PID.h"
#include "app_TimingCheck.h"
#include <stdlib.h>

/**	Main ��������ʱ��
 * ��Ϊ:	MainClock, SecondaryClock, DebugClock
 * MainClock��			10KHZ	��ʱ��. �ο���ϵͳ. AD7606_1,DAC8563ʱ��		(!)��Ҫ��AD7606_1��ȡ���֮����в���
 * SecondaryClock��	500HZ	��ʱ��. ������ϵͳ. 			 										(!)��Ҫ��AD7606_3��ȡ���֮����в���
 * DebugClock:			1HZ		����ʱ��. �����Ϣ.
 * 
 * */


uint8_t TestFlag = 1;

// ��������
void MainClockTask(void);
void SecondaryClock(void);
void DebugClockTask(void);


/* ������� */

//AD7606_x READY FLAG. ��Ϊʱ�Ӵ�����־.
	//FLAG_READY_1		AD7606_1��READY�źš�MainClock�Ĵ�����־.
	//FLAG_READY_3;		AD7606_3��READY�źš�SecondaryClock�Ĵ�����־.
extern CoilDriveFLAGS g_tCoilDriveFLAGS;

// ����Main�С����о������õ����������ͣ�Ӧ�������壿

//in bsp_motor_drive.c 	AD7606_1,_2,_3 ��ȡ���
extern AD7606_readings g_tReadings;

//in bsp_ADNS9800.c 	ADNS9800 ��ȡ���
extern ADNS9800_VAR_T g_tADNS9800;

//in app_Main_PID.c 	������̬
extern RotorOrientation g_tRotorOrientation;

//ϵͳֹͣ����Լ���ǰ����ģʽ
volatile _ctl_SYSTEM_CONTROL g_tSystemControl;

void System_Initializes(void)
{
	g_tSystemControl.STATUS = _ctl_STATUS_STOP;
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11*2)); 
	#endif
	BSP_Initializes();
	app_SubPID_Initializes();
	app_MainPID_Initializes();
}

void MainClockTask(void)
{
	if(!g_tCoilDriveFLAGS.FLAG_READY_1)
	{
			return;
	}
	
	_tim_TickTest();
	
	g_tCoilDriveFLAGS.FLAG_READY_1 = 0;
	_tim_TickMainTimer();
	SubPIDCalc();
	SubPIDExec();
}

void SecondaryClockTask(void)
{
	if(! (g_tCoilDriveFLAGS.FLAG_READY_3 && g_tCoilDriveFLAGS.FLAG_READY_2))
	{
		return;
	}
	g_tCoilDriveFLAGS.FLAG_READY_3 = 0;
	g_tCoilDriveFLAGS.FLAG_READY_2 = 0;
	_tim_TickSecondaryTimer();
	static uint32_t counter = 0;
	counter++;

	A9800OnTimerTask();

	if(!_cfg_DEBUG_MODE)
	{
		//TODO: CONTROL STUFF IN HERE
		//!!VENTUSFF
		g_tCoilDriveFLAGS.MainPID_Calculating = 1;
		MainPIDCalc();
		g_tCoilDriveFLAGS.MainPID_Calculating = 0;
		#if !_cfg_TEST_DECOUPLE && !_cfg_TEST_ORIENTATION
			MainPIDExec();
		#endif

	}
	else
	{
		
		//ʵ�ʼ��鷢�֣��ڼ���ʱ����Ӱ�쵽ԭ���Ѿ����õ�ʱ��
		//�����MainPID�ļ����в���MainTask();
		//���÷�ʽ������һ���µ�Flag: Main_Calculating��
			//��EXTI�ж���������Flag��������EXTI��Ҳִ��MainTask
		
		#if !_cfg_TEST_AMPLIFIER
			g_tCoilDriveFLAGS.MainPID_Calculating = 1;
			MainPIDCalc();
			g_tCoilDriveFLAGS.MainPID_Calculating = 0;
			#if !_cfg_TEST_DECOUPLE && !_cfg_TEST_ORIENTATION
				MainPIDExec();
			#endif
		#else
			static uint8_t toggle = 0;
			switch(toggle)
			{
				case 0:
				{
					_unit_SUB_PID_DATA tmp[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
					SubPIDSet(tmp);
					break;
				}
				case 1:
				{
					_unit_SUB_PID_DATA tmp[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
					SubPIDSet(tmp);
					break;
				}
				case 2:
				{
					_unit_SUB_PID_DATA tmp[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
					SubPIDSet(tmp);
					break;
				}
				case 3:
				{
					_unit_SUB_PID_DATA tmp[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
					SubPIDSet(tmp);
					break;
				}
			}

			
			if(toggle == 3)
			{
				toggle = 0;
			}
			else
			{
				toggle++;
			}
		#endif
		
		/*
		

		
		*/
		
	}
	
	if(counter == _tim_SECONDARY_TO_DEBUG)
	{
		counter = 0;
		DebugClockTask();
	}
}

void DebugClockTask(void)
{
	_tim_TickDebugTimer();
	if(_cfg_DEBUG_MODE)
	{
		if(!TestFlag)
		{
			TestFlag = 1;
		}
		
		char tmp[100];
		
		sprintf(tmp,"\nTime_Truth:%d  -  Debug:%d  -  Secondary:%d  -  Main:%d  -  Test:%d",
			_tim_GetGroundTruth(),_tim_GetDebugTimer(),_tim_GetSecondaryTimer(),_tim_GetMainTimer(),_tim_GetTestTimer());
		printx(tmp);
		
		printx("----------------------------------\n");
		
		#if !_cfg_TEST_ORIENTATION

		//���AD7606ͨ��
		for(uint8_t ii =0; ii<8;ii++)
		{
			sprintf(tmp,"CH%d:AD7606 reading: %lf V.",ii+1, g_tReadings.RAWarray[2][ii]*5.0/32768);
			printx(tmp);
		}
		
		sprintf(tmp,"\nMouse Sensor - 1: X:%d,  Y:%d,  DX:%d,  DY:%d",
			g_tADNS9800.PosX_1,g_tADNS9800.PosY_1,
			g_tADNS9800.MotionX_1,g_tADNS9800.MotionY_1);
		printx(tmp);
		sprintf(tmp,"Mouse Sensor - 2: X:%d,  Y:%d,  DX:%d,  DY:%d",
			g_tADNS9800.PosX_2,g_tADNS9800.PosY_2,
			g_tADNS9800.MotionX_2,g_tADNS9800.MotionY_2);
		printx(tmp);

		static _unit_MouseSensorINTData X_1 = 0,Y_1 = 0,X_2 = 0,Y_2 = 0;
		sprintf(tmp,"Caculated Cos(Theta2) = %f",
			MouseSensorCalculateCosTheta2(g_tADNS9800.PosX_1 - X_1,g_tADNS9800.PosY_1 - Y_1,
			g_tADNS9800.PosX_2 - X_2, g_tADNS9800.PosY_2 - Y_2));
		printx(tmp);
		X_1 = g_tADNS9800.PosX_1;
		Y_1 = g_tADNS9800.PosY_1;
		X_2 = g_tADNS9800.PosX_2;
		Y_2 = g_tADNS9800.PosY_2;

		_unit_MouseSensorFLOATData dtx,dty,dtz,error;
		error = MouseSensorToDTheta(g_tADNS9800.MotionX_1,g_tADNS9800.MotionY_1,
			g_tADNS9800.MotionX_2,g_tADNS9800.MotionY_2,&dtx,&dty,&dtz);
		if(_mth_abs(_mth_RAD_TO_ANGLE(dtz)) > 0.5f)
		{
		sprintf(tmp,"Calculated DThetaX: %7f, DThetaY: %7f, DThetaZ: %7f, \nERROR: %7f, ERROR Percentage:%2.1f%%",
			_mth_RAD_TO_ANGLE(dtx),_mth_RAD_TO_ANGLE(dty),_mth_RAD_TO_ANGLE(dtz),_mth_RAD_TO_ANGLE(error),
				_mth_abs(_mth_RAD_TO_ANGLE(error)/_mth_RAD_TO_ANGLE(dtz))*100);
		}
		else
		{
		sprintf(tmp,"Calculated DThetaX: %7f, DThetaY: %7f, DThetaZ: %7f",
			_mth_RAD_TO_ANGLE(dtx),_mth_RAD_TO_ANGLE(dty),_mth_RAD_TO_ANGLE(dtz));
		}
		printx(tmp);



		#endif
		sprintf(tmp,"\nCalculated Rotor Orientation: ThetaX: &%f&, ThetaY: &%f&, ThetaZ: &%f&",
			_mth_RAD_TO_ANGLE(g_tRotorOrientation.ThetaX), _mth_RAD_TO_ANGLE(g_tRotorOrientation.ThetaY),
			_mth_RAD_TO_ANGLE(g_tRotorOrientation.ThetaZ));
		printx(tmp);

		#if _cfg_TEST_ORIENTATION
		sprintf(tmp,"\nCalculated Rotor Rotation Matrix: ^%f^^%f^^%f^^%f^^%f^^%f^^%f^^%f^^%f^",g_tRotorOrientation.R_orientation[0],g_tRotorOrientation.R_orientation[1],
			g_tRotorOrientation.R_orientation[2],g_tRotorOrientation.R_orientation[3],g_tRotorOrientation.R_orientation[4],g_tRotorOrientation.R_orientation[5],
			g_tRotorOrientation.R_orientation[6],g_tRotorOrientation.R_orientation[7],g_tRotorOrientation.R_orientation[8]);
		printx(tmp);
		#endif
	}

}

int main(void)
{
	System_Initializes();
	DAC8563_ZeroData_Simultaneously();
	printx("System Initialize done.");

	while(1)
	{
		DAC8563_ZeroData_Simultaneously();
		while(g_tSystemControl.STATUS == _ctl_STATUS_STOP);
		printx("\nHello World From Ventus~!\n");
		printx("\n---------------------------------\nSpherical Motor Control System V1.0\n----------------------------------\nStarting in 1 second.\n");
		TIMDelay_Nms(1000);
		printx("System Started.");
		//��ʽ��ʼ����.
		TimingCheckStart();
		_bsp_StartMainClock();
		while(g_tSystemControl.STATUS == _ctl_STATUS_RUNNING)
		{
			//��g_tCoilDriveFLAGS.FLAG_READY_1�������ﲻ�У�
			//	��ÿһ��1ms�У���300us��ʱ�䶼������޷�ִ��main_clock

			MainClockTask();
			

			if(TestFlag)
			{
				_tim_StartTest();
			}
			//֮ǰ������else if��ֱ�ӵ���ִ�в�����
			//	��������SecondaryClockTask();Ƶ�������½�������1khz��������1/8 khz����.
			//������Ϊ����if֮��SecondaryClockTask()Ƶ��������������ȫ�������ǳ���ȷ
			SecondaryClockTask();
			
			if(TestFlag)
			{
				TestFlag = 0;
				_tim_StopTest();
			}
		}
	}

}


/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
