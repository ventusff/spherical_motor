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
    _unit_MAIN_PID_DATA F_k_0[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k)实际上就是要进行执行的结果

    _unit_MAIN_PID_DATA Kp[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Ki[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Kd[_cfg_MAIN_PID_CHANNEL_NUM];

    _unit_MAIN_PID_DATA K_2[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)的系数
    _unit_MAIN_PID_DATA K_1[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)的系数
    _unit_MAIN_PID_DATA K_0[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)的系数

    _unit_MAIN_PID_DATA target[_cfg_MAIN_PID_CHANNEL_NUM];

    //位置控制
    _unit_MAIN_PID_DATA target_Theta[3];
    _unit_MAIN_PID_DATA target_R[9];//目标姿态
    //轨迹规划
    uint8_t _flag_has_target;
    _unit_MAIN_PID_DATA Ro_axis_rotor[3];//从当前姿态到下一个轨迹规划点的轴
    _unit_MAIN_PID_DATA Ro_axis_stator[3];//n系下
    _unit_MAIN_PID_DATA M_K[9];//从当前姿态到下一个轨迹点的变换矩阵
    _unit_MAIN_PID_DATA R_K[9];//下一个轨迹点的姿态矩阵
    //_unit_MAIN_PID_DATA target[_cfg_MAIN_PID_CHANNEL_NUM]; //下一个轨迹点的RPY角


    //与位置控制p,i,d系数对应的角速度控制的pid系数；且均在K系下
    //PID系数：x,y方向是角速度控制,z向是位置控制
    /*
    _unit_MAIN_PID_DATA Kp_w[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Ki_w[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Kd_w[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA K_2_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)的系数
    _unit_MAIN_PID_DATA K_1_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)的系数
    _unit_MAIN_PID_DATA K_0_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)的系数
    //误差同样,x,y方向是角速度控制,z向是位置控制
    _unit_MAIN_PID_DATA e_k_2_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-2)
    _unit_MAIN_PID_DATA e_k_1_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k-1)
    _unit_MAIN_PID_DATA e_k_0_w[_cfg_MAIN_PID_CHANNEL_NUM];     //e(k)
    //输出：x,y方向是角速度控制,z向是位置控制
    _unit_MAIN_PID_DATA F_k_1_w[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k-1)
    _unit_MAIN_PID_DATA F_k_0_w[_cfg_MAIN_PID_CHANNEL_NUM];     //F(k)实际上就是要进行执行的结果
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
