#ifndef __APP_ROTOR_RATATE__H
#define __APP_ROTOR_RATATE__H

#include "configuration.h"
#include <math.h>
#include "bsp_ADNS9800.h"

typedef struct {
    //����ת������Ķ�����̬.3x3
    _unit_MAIN_PID_DATA R_orientation[3*3];
    _unit_MAIN_PID_DATA R_rotation[3*3];

    uint8_t _flag_update;
    _unit_MouseSensorINTData DX1, DY1, DX2, DY2;

    //������̬�ǣ�RPYֵ��(x:ThetaX, y:ThetaY, z:ThetaZ��˳��)  ����ڵ���̶ܹ�����ϵ�����Ӱ�����x,��y,��z˳����ת. 
    _unit_RotateAngleData ThetaX, ThetaY, ThetaZ; 

    //���ǵ������ԣ���֮ǰ����Ա�.��ѡ�����ȷ��
    _unit_RotateAngleData ThetaX_old, ThetaY_old, ThetaZ_old;    

    //������һʱ�̵���ת���󣨶���ϵ�£�
    _unit_MAIN_PID_DATA Ro_stator[3*3];

    //������һʱ�̵���ת���
    _unit_MAIN_PID_DATA Ro_axis_stator[3];
    _unit_MAIN_PID_DATA Ro_axis_rotor[3];
    _unit_MAIN_PID_DATA Ro_angle;

    //������һʱ�̵���ת���󣨶���ϵ�£�
    _unit_MAIN_PID_DATA Ro_rotor[3*3];
}RotorOrientation;

extern RotorOrientation g_tRotorOrientation;

/* �������ת�� */
_unit_MouseSensorFLOATData MouseSensorToDTheta(_unit_MouseSensorINTData dx1_i,_unit_MouseSensorINTData dy1_i,_unit_MouseSensorINTData dx2_i,_unit_MouseSensorINTData dy2_i,
    _unit_RotateAngleData* dThetaX, _unit_RotateAngleData* dThetaY, _unit_RotateAngleData* dThetaZ);
_unit_MouseSensorFLOATData MouseSensorINTToFloat(_unit_MouseSensorINTData x, _unit_MouseSensorFLOATData ScaleFactor);
_unit_MouseSensorFLOATData MouseSensorCalculateCosTheta2(
        _unit_MouseSensorINTData dx1_i,_unit_MouseSensorINTData dy1_i,
        _unit_MouseSensorINTData dx2_i,_unit_MouseSensorINTData dy2_i);

/* ��ת�����ʾ����̬������ڳ�ʼ״̬��RPY�˶���ʾ����̬��� */
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
