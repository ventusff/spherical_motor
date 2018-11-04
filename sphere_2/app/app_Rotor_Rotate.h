#ifndef __APP_ROTOR_RATATE__H
#define __APP_ROTOR_RATATE__H

#include "configuration.h"
#include <math.h>
#include "bsp_ADNS9800.h"

typedef struct {
    //由旋转矩阵定义的动子姿态.3x3
    _unit_MAIN_PID_DATA R_orientation[3*3];
    _unit_MAIN_PID_DATA R_rotation[3*3];

    uint8_t _flag_update;
    _unit_MouseSensorINTData DX1, DY1, DX2, DY2;

    //动子姿态角，RPY值。(x:ThetaX, y:ThetaY, z:ThetaZ的顺序)  相对于电机架固定坐标系，动子按照绕x,绕y,绕z顺序旋转. 
    _unit_RotateAngleData ThetaX, ThetaY, ThetaZ; 

    //考虑到两解性，与之前结果对比.来选择出正确解
    _unit_RotateAngleData ThetaX_old, ThetaY_old, ThetaZ_old;    

    //动子这一时刻的旋转矩阵（定子系下）
    _unit_MAIN_PID_DATA Ro_stator[3*3];

    //动子这一时刻的旋转轴角
    _unit_MAIN_PID_DATA Ro_axis_stator[3];
    _unit_MAIN_PID_DATA Ro_axis_rotor[3];
    _unit_MAIN_PID_DATA Ro_angle;

    //动子这一时刻的旋转矩阵（动子系下）
    _unit_MAIN_PID_DATA Ro_rotor[3*3];
}RotorOrientation;

extern RotorOrientation g_tRotorOrientation;

/* 鼠标数据转换 */
_unit_MouseSensorFLOATData MouseSensorToDTheta(_unit_MouseSensorINTData dx1_i,_unit_MouseSensorINTData dy1_i,_unit_MouseSensorINTData dx2_i,_unit_MouseSensorINTData dy2_i,
    _unit_RotateAngleData* dThetaX, _unit_RotateAngleData* dThetaY, _unit_RotateAngleData* dThetaZ);
_unit_MouseSensorFLOATData MouseSensorINTToFloat(_unit_MouseSensorINTData x, _unit_MouseSensorFLOATData ScaleFactor);
_unit_MouseSensorFLOATData MouseSensorCalculateCosTheta2(
        _unit_MouseSensorINTData dx1_i,_unit_MouseSensorINTData dy1_i,
        _unit_MouseSensorINTData dx2_i,_unit_MouseSensorINTData dy2_i);

/* 旋转矩阵表示的姿态、相对于初始状态的RPY运动表示的姿态求解 */
void app_UpdateRotorOrientation(void);
void app_InitRotorOrientation(void);
void app_GetRotorAngularVelocity(_unit_MAIN_PID_DATA* wx,_unit_MAIN_PID_DATA* wy, _unit_MAIN_PID_DATA* wz);
void app_StatorTorqueToRotorTorque(_unit_MAIN_PID_DATA Tx_s, _unit_MAIN_PID_DATA Ty_s, _unit_MAIN_PID_DATA Tz_s,
    _unit_MAIN_PID_DATA* Tx_r, _unit_MAIN_PID_DATA* Ty_r, _unit_MAIN_PID_DATA* Tz_r);
void update_RPY_angles(void);
void from_orientation_update_rotation(void);
void check_zero_device(void);

#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
