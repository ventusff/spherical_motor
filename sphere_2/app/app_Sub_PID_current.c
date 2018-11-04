#include "app_Sub_PID_current.h"
#include <stdlib.h>

/** ���ܣ��μ�����ϵͳ
 * ���ݶ�ȡ���ĵ���ֵ���Լ��趨�ĵ���Ŀ��ֵ��������ĵ�ѹ���п��ƣ���ʹ���������ܵ����Ͻ��.
 * PID �����+- 7.5V δ���Ŵ�ĵ�ѹ
 * PID ���룺Ŀ�����ֵ��ʵ�ʵ���ֵ.(��AD7606��ȡ���Ķ���/1ŷķ)
 * ���ض��󣺷Ŵ���+��Ȧ���.(2/(0.0001s+7))
 *  2:�Ŵ���;  0.0001s:100uH���;  7: 7ŷķ����
 * */

//ȫ�ֱ���
SubPID g_tSubPID;
extern AD7606_readings g_tReadings;

//�������������ѹ�����ұ�֤�������Ч��
void _mth_Satuation_SUB_PID_OUTPUT(_unit_SUB_PID_DATA* x)
{
    if(*x < 0.0f - _cfg_SUB_PID_OUTPUT_VOLTATE_MAGNITUDE)
        *x =  0.0f - _cfg_SUB_PID_OUTPUT_VOLTATE_MAGNITUDE;
    else if(*x > _cfg_SUB_PID_OUTPUT_VOLTATE_MAGNITUDE)
        *x =  _cfg_SUB_PID_OUTPUT_VOLTATE_MAGNITUDE;
    
    //��֤�����Ч��
    if(*x <= 0.0f - _cst_SUB_PID_OUTPUT_MAX)
    {
        *x = 0.0f - _cst_SUB_PID_OUTPUT_MAX + _cst_DAC_1_BIT_TO_V;
    }
}

//��DAC������ֵתΪ����ֵ
uint16_t _mth_DAC_FLOAT_To_UINT(_unit_SUB_PID_DATA x)
{
    return (uint16_t)((int16_t)(x * _cst_DAC_1_V_TO_BIT) + _cst_SUB_PID_OUTPUT_ZERO);
}

//��ADC����ֵתΪ������ֵ
_unit_SUB_PID_DATA _mth_ADC_UINT_To_FLOAT(int16_t x)
{
    return x * _cst_ADC_1_BIT_TO_V;
}

/**����������PIDĿ��
 * ���룺8·�����ĸ�����Ŀ�꣬��λA. ������ָ�룬n�������� (n == _cfg_SUB_PID_CHANNEL_NUM)
 * �������
 * */
void SubPIDSet(_unit_CURRENT_FLOAT_DATA* i_x8)
{
    if(_cfg_USE_SUB_PID)
    {
        for(uint8_t i = 0; i<_cfg_SUB_PID_CHANNEL_NUM; i++)
        {
            if(i_x8[i] < 0.0f - _cfg_SUB_PID_COIL_CURRENT_MAGNITUDE || i_x8[i] > _cfg_SUB_PID_COIL_CURRENT_MAGNITUDE)
            {
                return;
            }
            g_tSubPID.i_target[i] = i_x8[i];
        }
    }
    else
    {
        uint16_t out[_cfg_SUB_PID_CHANNEL_NUM];
        for(uint8_t i = 0;i<_cfg_SUB_PID_CHANNEL_NUM; i++)
        {
            out[i] = _mth_DAC_FLOAT_To_UINT(i_x8[i]*_cst_COIL_R/_cst_CURRENT_MULT);
        }

        //���
        DAC8563_SetData_Simultaneously(out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7]);
    }

}

//����PID����
void SubPIDCalc(void)
{
    #if _cfg_USE_SUB_PID
    {

        //static _unit_SUB_PID_DATA K2 = _par_SUB_PID_KP + _par_SUB_PID_KI + _par_SUB_PID_KD, K1 = 0.0 - _par_SUB_PID_KP - 2*_par_SUB_PID_KD, K0 = _par_SUB_PID_KD;

        for(uint8_t i = 0; i<_cfg_SUB_PID_CHANNEL_NUM; i++)
        {
            g_tSubPID.u_k_1[i] = g_tSubPID.u_k_0[i];
            g_tSubPID.e_k_2[i] = g_tSubPID.e_k_1[i];
            g_tSubPID.e_k_1[i] = g_tSubPID.e_k_0[i];

            //g_tSubPID.e_k_0[i] = g_tSubPID.i_target[i] - _mth_DAC_UINT_To_FLOAT(g_tReadings.RAWarray[AD7606_PORT1][i]);
            g_tSubPID.e_k_0[i] = g_tSubPID.i_target[i] - _mth_ADC_UINT_To_FLOAT(g_tReadings.RAWarray[AD7606_PORT1][7 - i]);

            g_tSubPID.u_k_0[i] = g_tSubPID.u_k_1[i]+
                g_tSubPID.K_0*g_tSubPID.e_k_0[i] + g_tSubPID.K_1*g_tSubPID.e_k_1[i] + g_tSubPID.K_2*g_tSubPID.e_k_2[i];
            
            /* �в�ͨ����Ϊ����˷���ʱ�ϳ�������10Khz ��MainTask ʱ�򱻴�����
            g_tSubPID.u_k_0[i] = g_tSubPID.u_k_1[i] 
                + _par_SUB_PID_KP*(g_tSubPID.e_k_0[i] - g_tSubPID.e_k_1[i]) 
                + _par_SUB_PID_KI*g_tSubPID.e_k_0[i]
                + _par_SUB_PID_KD*(g_tSubPID.e_k_0[i] - 2*g_tSubPID.e_k_1[i] + g_tSubPID.e_k_2[i]);
                */

            _mth_Satuation_SUB_PID_OUTPUT(&(g_tSubPID.u_k_0[i]));
        }
    }
    #else
        return;
    #endif

}

//ִ��PID������
void SubPIDExec(void)
{
    #if _cfg_USE_SUB_PID
    {
        uint16_t out[_cfg_SUB_PID_CHANNEL_NUM];
        for(uint8_t i = 0;i<_cfg_SUB_PID_CHANNEL_NUM; i++)
        {
            out[i] = _mth_DAC_FLOAT_To_UINT(g_tSubPID.u_k_0[i]);
        }

        //���
        DAC8563_SetData_Simultaneously(out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7]);
    }
    #else
        return;
    #endif
}

void app_SubPID_GetParameters(void)
{
    printx("-------- Please Enter SubPID Kp,Ki,Kd --------");
    char* str;
    char CutStr[3][20];
    uint8_t CutCount = 0, idx = 0;
    //uint16_t len;
    _unit_SUB_PID_DATA _Kp,_Ki,_Kd;
		str = USART1_GetLine();
    //�ַ����ָ�
    while(*str!='\0')
    {
        if(*str == ' ' || *str == ',' || *str == _cfg_USART_RECEIVE_END_FLAG)
        {
            CutStr[CutCount][idx] = '\0';
            CutCount++;
            idx = 0;
            if(*str == _cfg_USART_RECEIVE_END_FLAG)
                break;
        }
        else
        {
            CutStr[CutCount][idx] = *str;
            idx++;
        }
		str++;
    }

    _Kp = atof(CutStr[0]);
    _Ki = atof(CutStr[1]);
    _Kd = atof(CutStr[2]);


    g_tSubPID.Kp = _Kp;
    g_tSubPID.Ki = _Ki;
    g_tSubPID.Kd = _Kd;
}

//��ʼ��������
void app_SubPID_Initializes()
{
    
    printx("Initializing SubPID...");
    for(uint8_t i = 0; i < _cfg_SUB_PID_CHANNEL_NUM; i++)
    {
        g_tSubPID.i_target[i] = 0;
        g_tSubPID.e_k_0[i] = 0;
        g_tSubPID.e_k_1[i] = 0;
        g_tSubPID.e_k_2[i] = 0;
        g_tSubPID.u_k_1[i] = 0;
        g_tSubPID.u_k_0[i] = 0;
    }
		if(_cfg_USE_SUB_PID)
		{
				if(_cfg_USE_PRESET_PID)
				{
						g_tSubPID.Kp = _par_SUB_PID_KP;
						g_tSubPID.Ki = _par_SUB_PID_KI;
						g_tSubPID.Kd = _par_SUB_PID_KD;
				}
				else{
						app_SubPID_GetParameters();
				}
		}


    char tmp[50];
    sprintf(tmp,"SubPID: Kp = %3.1f, Ki = %3.1f, Kd = %3.1f",g_tSubPID.Kp,g_tSubPID.Ki,g_tSubPID.Kd);
		printx(tmp);

    g_tSubPID.K_0 = g_tSubPID.Kp + g_tSubPID.Ki + g_tSubPID.Kd;
    g_tSubPID.K_1 = 0.0f - g_tSubPID.Kp - 2*g_tSubPID.Kd;
    g_tSubPID.K_2 = g_tSubPID.Kd;

}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/

