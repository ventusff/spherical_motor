#ifndef __APP_DECOUPLE__H
#define __APP_DECOUPLE__H

#include "usart.h"
#include "configuration.h"
#include "app_Rotor_Rotate.h"
#include <math.h>

typedef struct{
    _unit_RotateAngleData Coil_Init_Position[3*_cfg_COIL_NUM];  //线圈初始位置.
    _unit_RotateAngleData Coil_Now_Position[3*_cfg_COIL_NUM];  //旋转后，根据动子新姿态，在动子坐标系下的线圈位置
    _unit_MAIN_PID_DATA VirtualCurrentK[_cfg_MAIN_PID_CHANNEL_NUM];
    _unit_MAIN_PID_DATA Coil_Txyz_Coeffs[_cfg_MAIN_PID_CHANNEL_NUM * _cfg_COIL_NUM];
    _unit_MAIN_PID_DATA matrix_Real_To_Virtual[_cfg_DAC_GROUP_NUM][_cfg_DAC_GROUP_NUM];
} MainPID_Decouple_T;

void app_Main_PID_Decouple(_unit_CURRENT_FLOAT_DATA* i_virtual, _unit_CURRENT_FLOAT_DATA* i_real);
void app_Main_PID_Decouple_Initializes(void);
void Get1CoilTxyzCoef(_unit_MAIN_PID_DATA x, _unit_MAIN_PID_DATA y, _unit_MAIN_PID_DATA z,
    _unit_MAIN_PID_DATA* kTx, _unit_MAIN_PID_DATA* kTy, _unit_MAIN_PID_DATA* kTz);
void SolveRealCurrentEquations(
        _unit_MAIN_PID_DATA _A[][_cfg_DAC_GROUP_NUM], _unit_CURRENT_FLOAT_DATA* x, const _unit_CURRENT_FLOAT_DATA* _b);
_unit_MAIN_PID_DATA detOf3x3(
    _unit_MAIN_PID_DATA a11,_unit_MAIN_PID_DATA a12,_unit_MAIN_PID_DATA a13,
    _unit_MAIN_PID_DATA a21,_unit_MAIN_PID_DATA a22,_unit_MAIN_PID_DATA a23,
    _unit_MAIN_PID_DATA a31,_unit_MAIN_PID_DATA a32,_unit_MAIN_PID_DATA a33);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
