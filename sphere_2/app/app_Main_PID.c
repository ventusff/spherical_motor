#include "app_Main_PID.h"
#include <stdlib.h>

/** 包含Main PID 主控制系统 实现
 * 
 * */
MainPID g_tMainPID;
_unit_CURRENT_FLOAT_DATA iVirtual[_cfg_MAIN_PID_CHANNEL_NUM] = {0.0,0.0,0.0};
_unit_CURRENT_FLOAT_DATA iReal[_cfg_DAC_CHANNEL_NUM];
extern _ctl_SYSTEM_CONTROL g_tSystemControl;

/** 函数：设置主PID控制目标
 * **/
void MainPIDSet(void)
{

}

void PID_update(_unit_MAIN_PID_DATA* input)
{
    for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
    {
        g_tMainPID.F_k_1[i] = g_tMainPID.F_k_0[i];
        g_tMainPID.e_k_2[i] = g_tMainPID.e_k_1[i];
        g_tMainPID.e_k_1[i] = g_tMainPID.e_k_0[i];

        //更新得到当前周期误差
        g_tMainPID.e_k_0[i] = g_tMainPID.target[i] - input[i];

        //PID
        g_tMainPID.F_k_0[i] = g_tMainPID.F_k_1[i]+
            g_tMainPID.K_0[i]*g_tMainPID.e_k_0[i] + g_tMainPID.K_1[i]*g_tMainPID.e_k_1[i] + g_tMainPID.K_2[i]*g_tMainPID.e_k_2[i];
    }
}

void interval_normal(_unit_MAIN_PID_DATA* ex)
{
    if(*ex<0.0f - _cst_PI)
        *ex += _cst_PI;
    else if(*ex>_cst_PI)
        *ex -= _cst_PI;
}

/** 函数：计算主PID
 * **/
void MainPIDCalc(void)
{

    //姿态更新：归零装置 / 鼠标芯片
    app_UpdateRotorOrientation();
    
    #if _cfg_TEST_ORIENTATION
    /*
    char tmp[100];
    sprintf(tmp,"\nCalculated Rotor Orientation: ThetaX: &%f&, ThetaY: &%f&, ThetaZ: &%f&",
        _mth_RAD_TO_ANGLE(g_tRotorOrientation.ThetaX), _mth_RAD_TO_ANGLE(g_tRotorOrientation.ThetaY),
        _mth_RAD_TO_ANGLE(g_tRotorOrientation.ThetaZ));
    printx(tmp);
    */
    #endif

    //根据所选择策略，计算所需力矩
    switch(g_tSystemControl.MODE)
    {
        case _ctl_MODE_ANGULAR_VELOCITY:
        {
            _unit_MAIN_PID_DATA w[3];
            app_GetRotorAngularVelocity(&(w[0]),&(w[1]),&(w[2]));
            PID_update(w);
            app_StatorTorqueToRotorTorque(g_tMainPID.F_k_0[0],g_tMainPID.F_k_0[1],g_tMainPID.F_k_0[2],&(iVirtual[0]),&(iVirtual[1]),&(iVirtual[2]));
            break;
        }

        case _ctl_MODE_ANGULAR_POSITION:
        {
            //获取erx,ery,erz
            //首先经过app_UpdateRotorOrientation();后 g_tRotorOrientation.ThetaX,Y,Z是更新过的
            _unit_MAIN_PID_DATA 
                erx = g_tMainPID.target_Theta[0] - g_tRotorOrientation.ThetaX,
                ery = g_tMainPID.target_Theta[1] - g_tRotorOrientation.ThetaY,
                erz = g_tMainPID.target_Theta[2] - g_tRotorOrientation.ThetaZ;
            interval_normal(&erx);
            interval_normal(&erx);
            interval_normal(&ery);
            interval_normal(&ery);
            interval_normal(&erz);
            interval_normal(&erz);
            

            //这个逻辑有问题：如果在震荡过程中超过了，有可能越来越严重
            /** 如果误差超过threshold
             *      如果没有事先的目标，就定义新系，并以新系下一近点为目标
             *      如果有事先的目标，就在原系下达到该目标为止，达到之后清除目标
             *  如果没有超过threshold，则直接以目标点为目标
            */

            //正确逻辑
            /** 如果没有事先的目标
            *       如果误差超过threshold，就定义新系，并以新系下一近点为目标
            *       如果没有超过threshold，则直接以目标点为目标
            *   如果有事先的目标，就在原系下达到该目标为止，达到之后清除目标
            * */
            if(!g_tMainPID._flag_has_target)
            {
                g_tMainPID._flag_has_target = 1;
                if( _mth_abs(erx)+_mth_abs(ery)+_mth_abs(erz) > _cfg_THRESHOLD * 2
                    || _mth_abs(erx) > _cfg_THRESHOLD || _mth_abs(ery) > _cfg_THRESHOLD || _mth_abs(erz) > _cfg_THRESHOLD)
                {
                    //归零PID误差
                    //这里可能有问题，应该需要考虑当前角速度作为切换时的角度误差的微分项
                    {
                        for(uint8_t i = 0; i < _cfg_MAIN_PID_CHANNEL_NUM; i++)
                        {
                            g_tMainPID.e_k_2[i] = 0;
                            g_tMainPID.e_k_1[i] = 0;
                            g_tMainPID.e_k_0[i] = 0;
                            g_tMainPID.F_k_0[i] = 0;
                            g_tMainPID.F_k_1[i] = 0;
                        }
                    }
                    //找到下一个轨迹规划点
                    {

                        //找到当前姿态到目标姿态的轴，定义角，求到当前姿态到轨迹规划点k的变换M（即C_b^k）
                        {
                            //C_b^b' = C_n^b'*C_b^n
                            //  C_b^n = C_n^b T 即当前姿态的旋转矩阵
                            //  C_n^b' = 旋转矩阵+RPY角计算出的 姿态矩阵

                            //C_b^b' = C_n^b'*C_b^n
                            _unit_MAIN_PID_DATA R_C_b_b_prime[3*3];
                            arm_matrix_instance_f32 pSrcA;
                            arm_matrix_instance_f32 pSrcB;
                            arm_matrix_instance_f32 pDst;

                            pSrcA.numCols = 3;
                            pSrcA.numRows = 3;
                            pSrcA.pData = g_tRotorOrientation.R_rotation;

                            pSrcB.numCols = 3;
                            pSrcB.numRows = 3;
                            pSrcB.pData = g_tMainPID.target_R;

                            pDst.numCols = 3;
                            pDst.numRows = 3;
                            pDst.pData = R_C_b_b_prime;

                            arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);

                            //由旋转矩阵反推轴角
                            //轴
                            _unit_MAIN_PID_DATA m1 = (R_C_b_b_prime[7] - R_C_b_b_prime[5]) * 0.5f,
                                m2 = (R_C_b_b_prime[2] - R_C_b_b_prime[6]) * 0.5f,
                                m3 = (R_C_b_b_prime[3] - R_C_b_b_prime[1]) * 0.5f; 
                            _unit_MAIN_PID_DATA s = (m1*m1+m2*m2+m3*m3);
                            _unit_MAIN_PID_DATA sqrt_s;
                            arm_sqrt_f32(s,&sqrt_s);
                            _unit_MAIN_PID_DATA sqrt_s_reciprocal = 1.0f/sqrt_s;

                            //注意！这个轴是b系下的！！！要换到n系下才行 C_b^n * axis_rotor = axis_stator
                            g_tMainPID.Ro_axis_rotor[0] = m1 * sqrt_s_reciprocal;
                            g_tMainPID.Ro_axis_rotor[1] = m2 * sqrt_s_reciprocal;
                            g_tMainPID.Ro_axis_rotor[2] = m3 * sqrt_s_reciprocal;

                            pSrcA.numCols = 3;
                            pSrcA.numRows = 3;
                            pSrcA.pData = g_tRotorOrientation.R_rotation;

                            pSrcB.numCols = 1;
                            pSrcB.numRows = 3;
                            pSrcB.pData = g_tMainPID.Ro_axis_rotor;

                            pDst.numCols = 1;
                            pDst.numRows = 3;
                            pDst.pData = g_tMainPID.Ro_axis_stator;

                            arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);

                            //角
                            //_unit_MAIN_PID_DATA c_ = (R_C_b_b_prime[0] + R_C_b_b_prime[4] + R_C_b_b_prime[8] - 1) * 0.5f;
                            //g_tMainPID.Ro_angle = acosf( c_ );
                            //_unit_MAIN_PID_DATA s_;
                            //arm_sqrt_f32((1.0f - c_ * c_), &s_);
                            _unit_MAIN_PID_DATA angle = _cfg_THRESHOLD, c_, s_;
                            c_ = arm_cos_f32(angle);
                            s_ = arm_sin_f32(angle);

                            //求到当前姿态到轨迹规划点k的变换M（即C_b^k）
                            _unit_MAIN_PID_DATA rx = g_tMainPID.Ro_axis_stator[0], ry = g_tMainPID.Ro_axis_stator[1],
                                rz = g_tMainPID.Ro_axis_stator[2];
                            _unit_MAIN_PID_DATA rx_1_minus_c_t = rx * (1.0f - c_),
                                                ry_1_minus_c_t = ry * (1.0f - c_),
                                                rz_1_minus_c_t = rz * (1.0f - c_),
                                                rx_s_t         = rx * s_,
                                                ry_s_t         = ry * s_,
                                                rz_s_t         = rz * s_;
                            g_tMainPID.M_K[0] = rx * rx_1_minus_c_t + c_;
                            g_tMainPID.M_K[1] = rx * ry_1_minus_c_t + rz_s_t;
                            g_tMainPID.M_K[2] = rx * rz_1_minus_c_t - ry_s_t;
                            g_tMainPID.M_K[3] = ry * rx_1_minus_c_t - rz_s_t;
                            g_tMainPID.M_K[4] = ry * ry_1_minus_c_t + c_;
                            g_tMainPID.M_K[5] = ry * rz_1_minus_c_t + rx_s_t;
                            g_tMainPID.M_K[6] = rz * rx_1_minus_c_t + ry_s_t;
                            g_tMainPID.M_K[7] = rz * ry_1_minus_c_t - rx_s_t;
                            g_tMainPID.M_K[8] = rz * rz_1_minus_c_t + c_;
                        }
                        //求到C_n^k = C_b^k * C_n^b
                        {
                            arm_matrix_instance_f32 pSrcA;
                            arm_matrix_instance_f32 pSrcB;
                            arm_matrix_instance_f32 pDst;

                            pSrcA.numCols = 3;
                            pSrcA.numRows = 3;
                            pSrcA.pData = g_tMainPID.M_K;

                            pSrcB.numCols = 3;
                            pSrcB.numRows = 3;
                            pSrcB.pData = g_tRotorOrientation.R_orientation;

                            pDst.numCols = 3;
                            pDst.numRows = 3;
                            pDst.pData = g_tMainPID.R_K;

                            arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);
                        }
                        //根据C_n^k反求出k的thetaX,Y,Z
                        {
                            //phi = ThetaZ = atan(ny,nx),phi = phi+180，注意phi的两解性
                            //这里要保证与当前姿态角的误差不超过pi！！
                            g_tMainPID.target[2] = atan2f(g_tMainPID.R_K[3],g_tMainPID.R_K[0]);
                            if(_mth_abs(g_tMainPID.target[2] - g_tRotorOrientation.ThetaZ)> _cfg_2_THRESHOLD)//TO INVESTIGATE
                            {
                                if(g_tMainPID.target[2]<g_tRotorOrientation.ThetaZ)
                                    g_tMainPID.target[2] += _cst_PI;
                                else
                                    g_tMainPID.target[2] -= _cst_PI;
                            }
                            /*
                            if(_mth_RAD_TO_ANGLE(_mth_abs(g_tMainPID.target[2] - g_tMainPID.target[2]_old)) 
                                > _cst_DELTA_ANGLE_EPSILONE_DEGREE )
                            {
                                g_tMainPID.target[2] = g_tMainPID.target[2] + _cst_PI;
                            }
                            */
                            _unit_MAIN_PID_DATA sinThetaZ = arm_sin_f32(g_tMainPID.target[2]), cosThetaZ = arm_cos_f32(g_tMainPID.target[2]);

                            //theta = ThetaY = atan2(-nz, cosphi*nx+sinphi*ny)

                            g_tMainPID.target[1] = atan2f(0.0f - g_tMainPID.R_K[6],
                                cosThetaZ * g_tMainPID.R_K[0] + sinThetaZ * g_tMainPID.R_K[3]);
                            if(_mth_abs(g_tMainPID.target[1] - g_tRotorOrientation.ThetaY)> _cfg_2_THRESHOLD )//TO INVESTIGATE
                            {
                                if(g_tMainPID.target[1]<g_tRotorOrientation.ThetaY)
                                    g_tMainPID.target[1] += _cst_PI;
                                else
                                    g_tMainPID.target[1] -= _cst_PI;
                            }
                            //psi = ThetaX = atan2(sinphi*ax-cosphi*ay, -sinphi*ox+cosphi*oy)
                            g_tMainPID.target[0] = atan2f(
                                sinThetaZ * g_tMainPID.R_K[2] - cosThetaZ * g_tMainPID.R_K[5],
                                cosThetaZ * g_tMainPID.R_K[4] - sinThetaZ * g_tMainPID.R_K[1]);
                            if(_mth_abs(g_tMainPID.target[0] - g_tRotorOrientation.ThetaX)> _cfg_2_THRESHOLD )//TO INVESTIGATE
                            {
                                if(g_tMainPID.target[0]<g_tRotorOrientation.ThetaX)
                                    g_tMainPID.target[0] += _cst_PI;
                                else
                                    g_tMainPID.target[0] -= _cst_PI;
                            }
                        }
                    }
                }
                else{//如果没有超过阈值，直接以最终目标点作为下一个轨迹规划点
                    for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
                    {
                        g_tMainPID.target[i] = g_tMainPID.target_Theta[i];
                    }
                }
            }

            //判断角度是否达到 到达阈值
            if(_mth_abs(erx)<_cfg_REACH_THRESHOLD 
                && _mth_abs(ery)<_cfg_REACH_THRESHOLD 
                && _mth_abs(erz)<_cfg_REACH_THRESHOLD)
            {
                g_tMainPID._flag_has_target = 0;
                printx("reach...\n");
            }
            else{
                _unit_MAIN_PID_DATA thetaNow[3];
                thetaNow[0] = g_tRotorOrientation.ThetaX;
                thetaNow[1] = g_tRotorOrientation.ThetaY;
                thetaNow[2] = g_tRotorOrientation.ThetaZ;
                PID_update(thetaNow);
                app_StatorTorqueToRotorTorque(g_tMainPID.F_k_0[0],g_tMainPID.F_k_0[1],g_tMainPID.F_k_0[2],&(iVirtual[0]),&(iVirtual[1]),&(iVirtual[2]));
            }


            break;
        }

        case _ctl_MODE_TORQUE_ROTOR_OUTPUT:
        {
            _unit_MAIN_PID_DATA target[3] = {0.0,0.0,8.0};
            for(uint8_t i = 0; i<3;i++)
            {
                iVirtual[i] = target[i];
            }
            break;
        }

        case _ctl_MODE_TORQUE_STATOR_OUTPUT:
        {
            _unit_MAIN_PID_DATA target[3] = {0.0,0.0,8.0};
            app_StatorTorqueToRotorTorque(target[0],target[1],target[2],&(iVirtual[0]),&(iVirtual[1]),&(iVirtual[2]));
            break;
        }
        default:
            break;
    }

    //输出所需力矩(动子系下)
    app_Main_PID_Decouple(iVirtual,iReal);
    #if _cfg_DEBUG_MODE
    {
        static uint16_t tick = 0;
        tick++;
        if(tick == 500)
        {
			tick = 0;
            char tmp[100];
            sprintf(tmp,"\nDecoupled i_Real: [%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f]",
                iReal[0],iReal[1],iReal[2],iReal[3],iReal[4],iReal[5],iReal[6],iReal[7]);
            printx(tmp);
        }
    }
    #endif
}

/** 函数：执行主PID
 * **/
void MainPIDExec(void)
{
	//_unit_SUB_PID_DATA tmp[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    SatuationMaxRealCurrentAndEnable();
	
    #if _cfg_DEBUG_MODE
    {
        static uint16_t tick = 0;
        tick++;
        if(tick == 500)
        {
			tick = 0;
            char tmp[100];
            sprintf(tmp,"Decoupled Output i_Real: [%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f]\n",
                iReal[0],iReal[1],iReal[2],iReal[3],iReal[4],iReal[5],iReal[6],iReal[7]);
            printx(tmp);
        }
    }
    #endif
	
    SubPIDSet(iReal);
}

/** 函数：利用等比例缩放，限制最大实际电流幅值。并且读取DAC的使能与不使能配置
 * */
void SatuationMaxRealCurrentAndEnable(void)
{
    _unit_CURRENT_FLOAT_DATA max = 0;
    for(uint8_t i = 0; i<_cfg_DAC_CHANNEL_NUM; i++)
    {
        if(_cfg_ENABLE_DAC[i]!=0)
        {
            _unit_CURRENT_FLOAT_DATA abs_i = _mth_abs(iReal[i]);
            if(abs_i>max)
            {
                max = abs_i;
            }
        }
        else{
            iReal[i] = 0;
        }

    }
    
    if(max > _cfg_SUB_PID_COIL_CURRENT_MAGNITUDE)
    {
        for(uint8_t i = 0; i<_cfg_DAC_CHANNEL_NUM; i++)
        {
            iReal[i] *= (_cfg_SUB_PID_COIL_CURRENT_MAGNITUDE/max);
        }
    }
}

void app_MainPID_Initializes(void)
{
    app_InitRotorOrientation();
    app_Main_PID_Decouple_Initializes();
    //获取当前运行模式
    g_tSystemControl.MODE = _ctl_MODE_ANGULAR_VELOCITY;
    //g_tSystemControl.MODE = _ctl_MODE_TORQUE_STATOR_OUTPUT;
    //g_tSystemControl.MODE = _ctl_MODE_ANGULAR_POSITION;

    if(g_tSystemControl.MODE == _ctl_MODE_ANGULAR_POSITION)
    {
        g_tMainPID._flag_has_target = 0;
        // 从RPY角计算回姿态矩阵 C_n^b'
        _unit_MAIN_PID_DATA 
            sindrx = arm_sin_f32(g_tMainPID.target_Theta[0]), cosdrx = arm_cos_f32(g_tMainPID.target_Theta[0]),
            sindry = arm_sin_f32(g_tMainPID.target_Theta[1]), cosdry = arm_cos_f32(g_tMainPID.target_Theta[1]),
            sindrz = arm_sin_f32(g_tMainPID.target_Theta[2]), cosdrz = arm_cos_f32(g_tMainPID.target_Theta[2]);
        g_tMainPID.target_R[0] = cosdry * cosdrz;
        g_tMainPID.target_R[1] = cosdrz * sindrx * sindry - cosdrx * sindrz;
        g_tMainPID.target_R[2] = sindrx * sindrz + cosdrx * cosdrz * sindry;
        g_tMainPID.target_R[3] = cosdry * sindrz;
        g_tMainPID.target_R[4] = cosdrx * cosdrz + sindrx * sindry * sindrz;
        g_tMainPID.target_R[5] = cosdrx * sindry * sindrz - cosdrz * sindrx;
        g_tMainPID.target_R[6] = 0.0f - sindry;
        g_tMainPID.target_R[7] = cosdry * sindrx;
        g_tMainPID.target_R[8] = cosdrx * cosdry;
    }

    for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
    {
        g_tMainPID.target[i] = 0.0;
    }

    if(g_tSystemControl.MODE != _ctl_MODE_TORQUE_ROTOR_OUTPUT && g_tSystemControl.MODE != _ctl_MODE_TORQUE_STATOR_OUTPUT)
    {
        //根据运行模式来初始化PID参数
        _unit_MAIN_PID_DATA kp_[3] = {2.0,2.0,2.0}, ki_[3] = {0.01,0.01,0.01}, kd_[3] = {0.0,0.0,0.0};

        if(_cfg_USE_PRESET_PID_MAIN)
        {
            for(uint8_t i = 0; i<3; i++)
            {
                g_tMainPID.Kp[i] = kp_[i];
                g_tMainPID.Ki[i] = ki_[i];
                g_tMainPID.Kd[i] = kd_[i];
            }
        }
        else{
            app_MainPID_GetParameters();
        }

        //计算增量式PID的各项系数
        for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
        {
            g_tMainPID.K_0[i] = g_tMainPID.Kp[i] + g_tMainPID.Ki[i] + g_tMainPID.Kd[i];
            g_tMainPID.K_1[i] = 0.0f - g_tMainPID.Kp[i] - 2*g_tMainPID.Kd[i];
            g_tMainPID.K_2[i] = g_tMainPID.Kd[i];
        }
    }

    g_tMainPID.target[2] = 4.0;
}

void app_MainPID_GetParameters(void)
{
    printx("-------- Please Enter MainPID Kp,Ki,Kd --------");
    char* str;
    char CutStr[3][20];
    uint8_t CutCount = 0, idx = 0;
    //uint16_t len;
    _unit_SUB_PID_DATA _Kp,_Ki,_Kd;
		str = USART1_GetLine();
    //字符串分割
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

    for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
    {
        g_tMainPID.Kp[i] = _Kp;
        g_tMainPID.Ki[i] = _Ki;
        g_tMainPID.Kd[i] = _Kd;
    }
}


/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
