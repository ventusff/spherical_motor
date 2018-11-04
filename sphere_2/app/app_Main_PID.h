#ifndef __APP_MAIN_PID__H
#define __APP_MAIN_PID__H

#include "configuration.h"
#include "usart.h"
#include "app_Sub_PID_current.h"
#include "app_Rotor_Rotate.h"
#include "app_Decouple.h"

typedef struct{
    _unit_MAIN_PID_DATA N_DOF_target[_cfg_MAIN_PID_CHANNEL_NUM];

    _unit_MAIN_PID_DATA e_k_2[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)
    _unit_MAIN_PID_DATA e_k_1[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)
    _unit_MAIN_PID_DATA e_k_0[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)
    
    _unit_MAIN_PID_DATA F_k_1[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k-1)
    _unit_MAIN_PID_DATA F_k_0[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k)ʵ���Ͼ���Ҫ����ִ�еĽ��

    _unit_MAIN_PID_DATA Kp[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Ki[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Kd[_cfg_MAIN_PID_CHANNEL_NUM];

    _unit_MAIN_PID_DATA K_2[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)��ϵ��
    _unit_MAIN_PID_DATA K_1[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)��ϵ��
    _unit_MAIN_PID_DATA K_0[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)��ϵ��

    _unit_MAIN_PID_DATA target[_cfg_MAIN_PID_CHANNEL_NUM];

    //λ�ÿ���
    _unit_MAIN_PID_DATA target_Theta[3];
    _unit_MAIN_PID_DATA target_R[9];//Ŀ����̬
    //�켣�滮
    uint8_t _flag_has_target;
    _unit_MAIN_PID_DATA Ro_axis_rotor[3];//�ӵ�ǰ��̬����һ���켣�滮�����
    _unit_MAIN_PID_DATA Ro_axis_stator[3];//nϵ��
    _unit_MAIN_PID_DATA M_K[9];//�ӵ�ǰ��̬����һ���켣��ı任����
    _unit_MAIN_PID_DATA R_K[9];//��һ���켣�����̬����
    //_unit_MAIN_PID_DATA target[_cfg_MAIN_PID_CHANNEL_NUM]; //��һ���켣���RPY��


    //��λ�ÿ���p,i,dϵ����Ӧ�Ľ��ٶȿ��Ƶ�pidϵ�����Ҿ���Kϵ��
    //PIDϵ����x,y�����ǽ��ٶȿ���,z����λ�ÿ���
    /*
    _unit_MAIN_PID_DATA Kp_w[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Ki_w[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Kd_w[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA K_2_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)��ϵ��
    _unit_MAIN_PID_DATA K_1_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)��ϵ��
    _unit_MAIN_PID_DATA K_0_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)��ϵ��
    //���ͬ��,x,y�����ǽ��ٶȿ���,z����λ�ÿ���
    _unit_MAIN_PID_DATA e_k_2_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)
    _unit_MAIN_PID_DATA e_k_1_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)
    _unit_MAIN_PID_DATA e_k_0_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)
    //�����x,y�����ǽ��ٶȿ���,z����λ�ÿ���
    _unit_MAIN_PID_DATA F_k_1_w[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k-1)
    _unit_MAIN_PID_DATA F_k_0_w[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k)ʵ���Ͼ���Ҫ����ִ�еĽ��
    */


} MainPID;

void app_MainPID_Initializes(void);
void MainPIDSet(void);
void MainPIDCalc(void);
void MainPIDExec(void);
void SatuationMaxRealCurrentAndEnable(void);
void PID_update(_unit_MAIN_PID_DATA* input);
void app_MainPID_GetParameters(void);

void interval_normal(_unit_MAIN_PID_DATA* ex);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
