#ifndef __APP_SUB_PID_CURRENT_H
#define __APP_SUB_PID_CURRENT_H

#include "bsp_motor_drive.h"
#include "usart.h"
#include "configuration.h"

typedef struct {
    _unit_SUB_PID_DATA i_target[_cfg_SUB_PID_CHANNEL_NUM];

    //_unit_SUB_PID_DATA i_1[_cfg_SUB_PID_CHANNEL_NUM];
    //_unit_SUB_PID_DATA i_0[_cfg_SUB_PID_CHANNEL_NUM];

    _unit_SUB_PID_DATA e_k_2[_cfg_SUB_PID_CHANNEL_NUM];     //e(k-2)
    _unit_SUB_PID_DATA e_k_1[_cfg_SUB_PID_CHANNEL_NUM];     //e(k-1)
    _unit_SUB_PID_DATA e_k_0[_cfg_SUB_PID_CHANNEL_NUM];     //e(k)
    
    _unit_SUB_PID_DATA u_k_1[_cfg_SUB_PID_CHANNEL_NUM];     //u(k-1)
    _unit_SUB_PID_DATA u_k_0[_cfg_SUB_PID_CHANNEL_NUM];     //u(k)实际上就是要进行执行的结果

    _unit_SUB_PID_DATA Kp;
    _unit_SUB_PID_DATA Ki;
    _unit_SUB_PID_DATA Kd;

    _unit_SUB_PID_DATA K_2;     //e(k-2)的系数
    _unit_SUB_PID_DATA K_1;     //e(k-1)的系数
    _unit_SUB_PID_DATA K_0;     //e(k)的系数
} SubPID;

void SubPIDSet(_unit_CURRENT_FLOAT_DATA* i_x8);
void SubPIDCalc(void);
void SubPIDExec(void);

void app_SubPID_Initializes(void);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
