#include "app_Rotor_Rotate.h"

RotorOrientation g_tRotorOrientation;
extern ADNS9800_VAR_T g_tADNS9800;
extern USART_GetLine g_tUSART_2_GetLine;

void check_zero_device(void)
{

}

void app_GetRotorAngularVelocity(_unit_MAIN_PID_DATA* wx,_unit_MAIN_PID_DATA* wy, _unit_MAIN_PID_DATA* wz)
{
    _unit_RotateAngleData drx,dry,drz;
    MouseSensorToDTheta(g_tADNS9800.MotionX_1,g_tADNS9800.MotionY_1,
        g_tADNS9800.MotionX_2,g_tADNS9800.MotionY_2,&drx,&dry,&drz);

    //wx = drx / _cst_SECONDARY_CLOCK_SECONDS = drx * _cfg_SECONDARY_CLOCK
    *wx = drx * _cfg_SECONDARY_CLOCK;
    *wy = dry * _cfg_SECONDARY_CLOCK;
    *wz = drz * _cfg_SECONDARY_CLOCK;
}

void app_UpdateRotorOrientation(void)
{

    #if _cfg_USE_ZERO_DEVICE
        //如果归零装置有数据传来，那么本周期内不进行归零装置解算，直接利用归零装置数据
        if(g_tUSART_2_GetLine.ReadyFlag)
        {
            //safety && data complete check
            if(g_tUSART_2_GetLine.ReceiveCount == 0 && g_tUSART_2_GetLine.strLen == 16)
            {
                if(g_tUSART_2_GetLine.receive_buffer[0] == '^' && g_tUSART_2_GetLine.receive_buffer[13] == '^')
                {
                    //不要忘记清空鼠标芯片所有累积数据
                    g_tRotorOrientation.DX1 = 0;
                    g_tRotorOrientation.DY1 = 0;
                    g_tRotorOrientation.DX2 = 0;
                    g_tRotorOrientation.DY2 = 0;

                    uint8_t* buf_ = g_tUSART_2_GetLine.receive_buffer;
                    uint32_t data1 = (buf_[1] << 24) | (buf_[2] << 16) | (buf_[3] << 8) | buf_[4];
                    uint32_t data2 = (buf_[5] << 24) | (buf_[6] << 16) | (buf_[7] << 8) | buf_[8];
                    uint32_t data3 = (buf_[9] << 24) | (buf_[10] << 16) | (buf_[11] << 8) | buf_[12];
                    float32_t* drx_p = (float32_t*) (&data1);
                    float32_t* dry_p = (float32_t*) (&data2);
                    float32_t* drz_p = (float32_t*) (&data3);
                    _unit_RotateAngleData drx = *drx_p;
                    _unit_RotateAngleData dry = *dry_p;
                    _unit_RotateAngleData drz = *drz_p;

                    // 从RPY角计算回姿态矩阵
                    _unit_MAIN_PID_DATA 
                        sindrx = arm_sin_f32(drx), cosdrx = arm_cos_f32(drx),
                        sindry = arm_sin_f32(dry), cosdry = arm_cos_f32(dry),
                        sindrz = arm_sin_f32(drz), cosdrz = arm_cos_f32(drz);
                    g_tRotorOrientation.R_orientation[0] = cosdry * cosdrz;
                    g_tRotorOrientation.R_orientation[1] = cosdrz * sindrx * sindry - cosdrx * sindrz;
                    g_tRotorOrientation.R_orientation[2] = sindrx * sindrz + cosdrx * cosdrz * sindry;
                    g_tRotorOrientation.R_orientation[3] = cosdry * sindrz;
                    g_tRotorOrientation.R_orientation[4] = cosdrx * cosdrz + sindrx * sindry * sindrz;
                    g_tRotorOrientation.R_orientation[5] = cosdrx * sindry * sindrz - cosdrz * sindrx;
                    g_tRotorOrientation.R_orientation[6] = 0.0f - sindry;
                    g_tRotorOrientation.R_orientation[7] = cosdry * sindrx;
                    g_tRotorOrientation.R_orientation[8] = cosdrx * cosdry;

                    from_orientation_update_rotation();

                    // 直接更新RPY角
                    g_tRotorOrientation.ThetaX = drx;
                    g_tRotorOrientation.ThetaY = dry;
                    g_tRotorOrientation.ThetaZ = drz;


                    g_tUSART_2_GetLine.ReadyFlag = 0;
                    return;
                }
            }
        }
    #endif

    //根据鼠标芯片读数更新动子姿态
    //姿态角用于：1.PID更新计算误差值 2.由虚电流推出实电流(利用app_Decouple.c)
    {
        g_tRotorOrientation._flag_update = 0;
        
        g_tRotorOrientation.DX1 += g_tADNS9800.MotionX_1;
        g_tRotorOrientation.DY1 += g_tADNS9800.MotionY_1;
        g_tRotorOrientation.DX2 += g_tADNS9800.MotionX_2;
        g_tRotorOrientation.DY2 += g_tADNS9800.MotionY_2;

        _unit_MouseSensorFLOATData drx,dry,drz;
		MouseSensorToDTheta(g_tRotorOrientation.DX1,g_tRotorOrientation.DY1,
			g_tRotorOrientation.DX2,g_tRotorOrientation.DY2,&drx,&dry,&drz);

        // 由欧拉角得出旋转矩阵
        // 由于这个是直接测到的球的旋转，实质就是上是坐标系的旋转
        // 正常情况下，下列运算耗时估计：6*0.5+21*0.1 = 5.1 us  //即使不使用DSP库也不使用硬件浮点，估计耗时： 120 ~ 200us左右
        // 传输9*4位8bit数据串口用时最短估计： 2.5ms
        // 传输3*4位8bit数据串口用时最短估计： 833.3us
        // 可以看到，计算相较于通讯的耗时是非常少的。
        // 因此对于最终数据的更新，采用在计算机端解算为RPY值后重新计算回姿态矩阵并更新

        _unit_MAIN_PID_DATA 
            sindrx = arm_sin_f32(drx), cosdrx = arm_cos_f32(drx),
            sindry = arm_sin_f32(dry), cosdry = arm_cos_f32(dry),
            sindrz = arm_sin_f32(drz), cosdrz = arm_cos_f32(drz);
        g_tRotorOrientation.Ro_stator[0] = cosdry * cosdrz;
        g_tRotorOrientation.Ro_stator[1] = cosdrz * sindrx * sindry - cosdrx * sindrz;
        g_tRotorOrientation.Ro_stator[2] = sindrx * sindrz + cosdrx * cosdrz * sindry;
        g_tRotorOrientation.Ro_stator[3] = cosdry * sindrz;
        g_tRotorOrientation.Ro_stator[4] = cosdrx * cosdrz + sindrx * sindry * sindrz;
        g_tRotorOrientation.Ro_stator[5] = cosdrx * sindry * sindrz - cosdrz * sindrx;
        g_tRotorOrientation.Ro_stator[6] = 0.0f - sindry;
        g_tRotorOrientation.Ro_stator[7] = cosdry * sindrx;
        g_tRotorOrientation.Ro_stator[8] = cosdrx * cosdry;

        //由旋转矩阵反推轴角
        //轴
        _unit_MAIN_PID_DATA m1 = (g_tRotorOrientation.Ro_stator[7] - g_tRotorOrientation.Ro_stator[5]) * 0.5f,
            m2 = (g_tRotorOrientation.Ro_stator[2] - g_tRotorOrientation.Ro_stator[6]) * 0.5f,
            m3 = (g_tRotorOrientation.Ro_stator[3] - g_tRotorOrientation.Ro_stator[1]) * 0.5f; 
        _unit_MAIN_PID_DATA s = (m1*m1+m2*m2+m3*m3);
        //1e-5过大！这里进行一个累积
        //在DY = 1的量级下，s = 1.7e-8
        //在DY = 3的量级下，s = 1.6e-7
        //在DY = 9的量级下，s = 1.4e-6
        //实验中可以发现，鼠标芯片自身波动即可达到DY = 3, s = 1.6e-7的数量级
        if(s<1e-3f)
        {
            //如果并未发生明显旋转
            return;
        }
        //如果发生了，进入计算并且发生一次更新
        g_tRotorOrientation.DX1 = 0;
        g_tRotorOrientation.DY1 = 0;
        g_tRotorOrientation.DX2 = 0;
        g_tRotorOrientation.DY2 = 0;
        g_tRotorOrientation._flag_update = 1;

        //issue: 角速度解算
        //因为在这里发生了姿态变更，因此利用鼠标芯片数据更新当前角速度
        //  考虑：鼠标芯片虽有累积误差，但是具体每一时刻的数据认为其还是准确的
        //  方式1：直接利用每次鼠标芯片的单周期微元数据，直接计算角速度；
        //      直接写在鼠标芯片解算中
        //  方式2：利用这里累积之后认为有效的多周期累积微元数据，计算角速度
        //      写在这里
        //  现采用方式1


        _unit_MAIN_PID_DATA sqrt_s;
        arm_sqrt_f32(s,&sqrt_s);
        _unit_MAIN_PID_DATA sqrt_s_reciprocal = 1.0f/sqrt_s;
        g_tRotorOrientation.Ro_axis_stator[0] = m1 * sqrt_s_reciprocal;
        g_tRotorOrientation.Ro_axis_stator[1] = m2 * sqrt_s_reciprocal;
        g_tRotorOrientation.Ro_axis_stator[2] = m3 * sqrt_s_reciprocal;
        //角
        _unit_MAIN_PID_DATA c_ = (g_tRotorOrientation.Ro_stator[0] + g_tRotorOrientation.Ro_stator[4] + g_tRotorOrientation.Ro_stator[8] - 1) * 0.5f;
        //可以看出，在利用acosf	求余弦时对于非常小的角度已经累积了有一点大的误差
        //e.g.: 真实角度 = 0.019542, 实际ARM计算 = 0.0218315758
        //计算中的cos值有1e-5的误差，对于acos会带来3e-3左右的误差
        /** 计算中可以发现：
         * 对于比较小的旋转角度，例如：angle = 0.00088269 (rx = -0.00079999998, s 在 7e-7级别)
         * 而acosf有着固定的3e-3左右的误差，已经远大于角度此时本来的8e-4的数量级
         *      而观察中可以看到sin,cos包括旋转矩阵的计算对于这样小的角误差都没有特别大
         * 如果坚持用acosf，则至少需要保持角度在3e-2的级别----这事实上已经不满足我们的要求了
         * 因此，在这里抛弃acosf，直接利用sin开根的性质求 (既然angle值其实不是直接使用的)
         *      但是又观察到：c_的值本身就会有 5e-5量级的误差，而s = 1.5e-7带来的c_误差需求在1e-8以上
         * 因此：目前采取做法：累积鼠标芯片变化值，直到s 达到 1e-5的量级
         * 
         * 实验1：s累计到1e-5才更新。
         *      仍然发现，c_具有5e-5的误差，而实际1-c_ = 8.7e-6
         *      而s_ 也具有4e-3级别的误差
         * 实验2：s累计到1e-4才更新。角度量级：0.012，鼠标芯片数值量级：100
         *      s_ 达到了0.012的量级，误差2e-3，可以接受;
         *      c_ 的量级仍在1e-5以内，实测具有1e-4的误差
         *      Ro_rotor 普遍具有1e-3级别的误差，然而R_rotor的量级在1e-2~1e-3之间
         *      因此，还是不够
         * 实验3：s累计到1e-3才更新。角度量级：0.033 鼠标芯片数值量级：200~300
         *      s 误差在1e-7以内
         *      c_: 0.99947，算得0.99942, 5e-5误差
         *      s_: 0.03266, 算得0.03381, 1e-3误差
         *      Ro_rotor: 量级在1e-2，具有4e-4级别的误差
         *      大致可以接受
         * */
        //g_tRotorOrientation.Ro_angle = acosf( c_ );
        //_unit_MAIN_PID_DATA s_ = arm_sin_f32(g_tRotorOrientation.Ro_angle);
        _unit_MAIN_PID_DATA s_;
        arm_sqrt_f32((1.0f - c_ * c_), &s_);

        //求出当前轴在动子系下的坐标
        arm_matrix_instance_f32 pSrcA;
        arm_matrix_instance_f32 pSrcB;
        arm_matrix_instance_f32 pDst;

        pSrcA.numCols = 3;
        pSrcA.numRows = 3;
        pSrcA.pData = g_tRotorOrientation.R_orientation;

        pSrcB.numCols = 1;
        pSrcB.numRows = 3;
        pSrcB.pData = g_tRotorOrientation.Ro_axis_stator;

        pDst.numCols = 1;
        pDst.numRows = 3;
        pDst.pData = g_tRotorOrientation.Ro_axis_rotor;

        arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);


        //根据当前轴在动子系下坐标，反推出动子系下的旋转矩阵
        _unit_MAIN_PID_DATA rx = g_tRotorOrientation.Ro_axis_rotor[0], ry = g_tRotorOrientation.Ro_axis_rotor[1],
            rz = g_tRotorOrientation.Ro_axis_rotor[2];
        _unit_MAIN_PID_DATA rx_1_minus_c_t = rx * (1.0f - c_),
                            ry_1_minus_c_t = ry * (1.0f - c_),
                            rz_1_minus_c_t = rz * (1.0f - c_),
                            rx_s_t         = rx * s_,
                            ry_s_t         = ry * s_,
                            rz_s_t         = rz * s_;
        g_tRotorOrientation.Ro_rotor[0] = rx * rx_1_minus_c_t + c_;
        g_tRotorOrientation.Ro_rotor[1] = rx * ry_1_minus_c_t + rz_s_t;
        g_tRotorOrientation.Ro_rotor[2] = rx * rz_1_minus_c_t - ry_s_t;
        g_tRotorOrientation.Ro_rotor[3] = ry * rx_1_minus_c_t - rz_s_t;
        g_tRotorOrientation.Ro_rotor[4] = ry * ry_1_minus_c_t + c_;
        g_tRotorOrientation.Ro_rotor[5] = ry * rz_1_minus_c_t + rx_s_t;
        g_tRotorOrientation.Ro_rotor[6] = rz * rx_1_minus_c_t + ry_s_t;
        g_tRotorOrientation.Ro_rotor[7] = rz * ry_1_minus_c_t - rx_s_t;
        g_tRotorOrientation.Ro_rotor[8] = rz * rz_1_minus_c_t + c_;

        if(drz > 0.06f)
        {
            uint8_t jjj = 0;
            jjj++;
        }

        //从动子轴角导出的旋转矩阵来更新姿态
        pSrcA.numCols = 3;
        pSrcA.numRows = 3;
        pSrcA.pData = g_tRotorOrientation.Ro_rotor;

        pSrcB.numCols = 3;
        pSrcB.numRows = 3;
        pSrcB.pData = g_tRotorOrientation.R_orientation;

        _unit_MAIN_PID_DATA pDstData[3*3];

        pDst.numCols = 3;
        pDst.numRows = 3;
        pDst.pData = pDstData;

		arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);

        for(uint8_t i = 0; i< 9;i++)
        {
            g_tRotorOrientation.R_orientation[i] = pDstData[i];
        }

        from_orientation_update_rotation();

        update_RPY_angles();
    }  
}

void app_StatorTorqueToRotorTorque(_unit_MAIN_PID_DATA Tx_s, _unit_MAIN_PID_DATA Ty_s, _unit_MAIN_PID_DATA Tz_s,
    _unit_MAIN_PID_DATA* Tx_r, _unit_MAIN_PID_DATA* Ty_r, _unit_MAIN_PID_DATA* Tz_r)
{
    _unit_MAIN_PID_DATA pDataB[3] = {0.0,0.0,0.0};
    _unit_MAIN_PID_DATA pDataDst[3] = {0.0,0.0,0.0};

    pDataB[0] = Tx_s;
    pDataB[1] = Ty_s;
    pDataB[2] = Tz_s;

    arm_matrix_instance_f32 pSrcA;
    arm_matrix_instance_f32 pSrcB;
    arm_matrix_instance_f32 pDst;

    pSrcA.numCols = 3;
    pSrcA.numRows = 3;
    pSrcA.pData = g_tRotorOrientation.R_orientation;

    pSrcB.numCols = 1;
    pSrcB.numRows = 3;
    pSrcB.pData = pDataB;

    pDst.numCols = 1;
    pDst.numRows = 3;
    pDst.pData = pDataDst;

    arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);

    *Tx_r = pDataDst[0];
    *Ty_r = pDataDst[1];
    *Tz_r = pDataDst[2];
}

void from_orientation_update_rotation(void)
{
    arm_matrix_instance_f32 pSrcA;
    arm_matrix_instance_f32 pDst;

    //根据动子姿态矩阵转置得到旋转矩阵
    pSrcA.numCols = 3;
    pSrcA.numRows = 3;
    pSrcA.pData = g_tRotorOrientation.R_orientation;

    pDst.numCols = 3;
    pDst.numRows = 3;
    pDst.pData = g_tRotorOrientation.R_rotation;

    arm_mat_trans_f32(&pSrcA,&pDst);
}

void update_RPY_angles(void)
{
    //根据姿态拆分计算出当前 psi,theta,phi(绕x，绕y，绕z)
    //姿态不是直接相加旋转角的。要根据左乘上述旋转之后的新姿态矩阵来反推x,y,z
    //方法：用RPY组合变换表示运动；逆运动学解法之：RPY变换解法
    //!!VENTUSFF 注意此处有第二解！
    //  可以考虑在尝试时500Hz的主控频率下，一个周期角度最大最大变化仅5~10度左右
    {
        g_tRotorOrientation.ThetaX_old = g_tRotorOrientation.ThetaX;
        g_tRotorOrientation.ThetaY_old = g_tRotorOrientation.ThetaY;
        g_tRotorOrientation.ThetaZ_old = g_tRotorOrientation.ThetaZ;

        //phi = ThetaZ = atan(ny,nx),phi = phi+180，注意phi的两解性
        g_tRotorOrientation.ThetaZ = atan2f(g_tRotorOrientation.R_orientation[3],g_tRotorOrientation.R_orientation[0]);
        /*
        if(_mth_RAD_TO_ANGLE(_mth_abs(g_tRotorOrientation.ThetaZ - g_tRotorOrientation.ThetaZ_old)) 
            > _cst_DELTA_ANGLE_EPSILONE_DEGREE )
        {
            g_tRotorOrientation.ThetaZ = g_tRotorOrientation.ThetaZ + _cst_PI;
        }
        */
        _unit_MAIN_PID_DATA sinThetaZ = arm_sin_f32(g_tRotorOrientation.ThetaZ), cosThetaZ = arm_cos_f32(g_tRotorOrientation.ThetaZ);

        //theta = ThetaY = atan2(-nz, cosphi*nx+sinphi*ny)

        g_tRotorOrientation.ThetaY = atan2f(0.0f - g_tRotorOrientation.R_orientation[6],
             cosThetaZ * g_tRotorOrientation.R_orientation[0] + sinThetaZ * g_tRotorOrientation.R_orientation[3]);

        //psi = ThetaX = atan2(sinphi*ax-cosphi*ay, -sinphi*ox+cosphi*oy)
        g_tRotorOrientation.ThetaX = atan2f(
            sinThetaZ * g_tRotorOrientation.R_orientation[2] - cosThetaZ * g_tRotorOrientation.R_orientation[5],
            cosThetaZ * g_tRotorOrientation.R_orientation[4] - sinThetaZ * g_tRotorOrientation.R_orientation[1]);
    }
}

void app_InitRotorOrientation(void)
{
    _unit_MAIN_PID_DATA RotorInitOrientation[3*3] = {1,0,0,0,1,0,0,0,1};
    for(uint8_t i = 0; i < 9; i++)
    {
        g_tRotorOrientation.R_orientation[i] = RotorInitOrientation[i];
    }

    arm_matrix_instance_f32 pSrcA;
    arm_matrix_instance_f32 pDst;

    pSrcA.numRows = 3;
    pSrcA.numCols = 3;
    pSrcA.pData = g_tRotorOrientation.R_orientation;

    pDst.numRows = 3;
    pDst.numCols = 3;
    pDst.pData = g_tRotorOrientation.R_rotation;

    arm_mat_trans_f32(&pSrcA, &pDst);

    g_tRotorOrientation.ThetaX = 0;
    g_tRotorOrientation.ThetaY = 0;
    g_tRotorOrientation.ThetaZ = 0;
    g_tRotorOrientation.ThetaX_old = 0;
    g_tRotorOrientation.ThetaY_old = 0;
    g_tRotorOrientation.ThetaZ_old = 0;

    g_tRotorOrientation._flag_update = 0;
    g_tRotorOrientation.DX1 = 0;
    g_tRotorOrientation.DY1 = 0;
    g_tRotorOrientation.DX2 = 0;
    g_tRotorOrientation.DY2 = 0;
}


/**函数：将鼠标芯片的整型数据转换为浮点型数据
 * 输入：鼠标芯片整型数据x, 缩放因子（即考虑到表面非完全相切情况下的变形）(默认：1.0)，配置参数 _cfg_ADNS9800CPI
 * 输出：鼠标芯片浮点型数据，单位mm
 * */
_unit_MouseSensorFLOATData MouseSensorINTToFloat(_unit_MouseSensorINTData x, _unit_MouseSensorFLOATData ScaleFactor)
{
    //提速
    static _unit_MouseSensorFLOATData CPI_to_mm = 25.4/_cfg_ADNS9800_CPI;
    //1 inch = 25.4mm，故此单位为mm
    return ((_unit_MouseSensorFLOATData)x)*CPI_to_mm*ScaleFactor;
}

/**函数：将鼠标芯片浮点数据dx1,dy1,dx2,dy2解算为dthetax,dthetay,dthetaz三旋转角
 * 输入：
 * 1.dx1,dy1,dx2,dy2 鼠标芯片原始整型数据
 * 2.dtx,dty,dtz 解算出的三旋转角，单位：rad
 * 输出：dThetaZ 两种算法的差值
 * */
_unit_MouseSensorFLOATData MouseSensorToDTheta(_unit_MouseSensorINTData dx1_i,_unit_MouseSensorINTData dy1_i,_unit_MouseSensorINTData dx2_i,_unit_MouseSensorINTData dy2_i,
    _unit_RotateAngleData* dThetaX, _unit_RotateAngleData* dThetaY, _unit_RotateAngleData* dThetaZ)
{
    _unit_MouseSensorFLOATData dx1,dy1,dx2,dy2;
    _unit_MouseSensorFLOATData dThetaZ_1,dThetaZ_2;

    dx1 = MouseSensorINTToFloat(dx1_i,1.0);
    dy1 = MouseSensorINTToFloat(dy1_i,1.0);
    dx2 = MouseSensorINTToFloat(dx2_i,1.0);
    dy2 = MouseSensorINTToFloat(dy2_i,1.0);

    /* 除法耗时说明
     * 因为ARM体系结构本身并不包含除法运算硬件，所以在ARM上实现除法是十分耗时的。
     * ARM指令集中没有直接提供除法汇编指令，当代码中出现除法运算时，ARM编译器会调用C库函数
     * （有符合除法调用_rt_sdiv，无符合除法调用_rt_udiv），来实现除法操作。
     * 根据除数和被除数的不同，32bit的除法运算一般要占有20－140个指令周期。
     * **/

    //*dThetaX = dy2/(0.0-_cst_ROTOR_RADIUS);//除法耗时较长，换作乘法
    //*dThetaY = dy1/_cst_ROTOR_RADIUS;
    //dThetaZ_1 = (dx1 - dy2*_cst_COS_MOUSE_SENSOR_THETA)/_cst_ROTOR_RADIUS_X_MOUSE_SIN_THETA;
    //dThetaZ_2 = (dx2 + dy1*_cst_COS_MOUSE_SENSOR_THETA)/_cst_ROTOR_RADIUS_X_MOUSE_SIN_THETA;

    *dThetaX = dy2 * (0.0f - _cst_ROTOR_RADIUS_RECIPROCAL);
    *dThetaY = dy1 * _cst_ROTOR_RADIUS_RECIPROCAL;

    dThetaZ_1 = (dx1 - dy2*_cst_COS_MOUSE_SENSOR_THETA) * _cst_ROTOR_RADIUS_X_MOUSE_SIN_THETA_RECIPROCAL;
    dThetaZ_2 = (dx2 + dy1*_cst_COS_MOUSE_SENSOR_THETA) * _cst_ROTOR_RADIUS_X_MOUSE_SIN_THETA_RECIPROCAL;
    *dThetaZ = (dThetaZ_1 + dThetaZ_2) * 0.5f;

    return (dThetaZ_1 - dThetaZ_2);
}

/**函数：由鼠标芯片浮点数据dx1,dy1,dx2,dy2解算出cos(theta2)
 * 输入：dx1,dy1,dx2,dy2 鼠标芯片原始整型数据
 * 输出：cos(theta2) 计算参数
 * */
_unit_MouseSensorFLOATData MouseSensorCalculateCosTheta2(
        _unit_MouseSensorINTData dx1_i,_unit_MouseSensorINTData dy1_i,
        _unit_MouseSensorINTData dx2_i,_unit_MouseSensorINTData dy2_i)
{
    _unit_MouseSensorFLOATData dx1,dy1,dx2,dy2;
    _unit_RotateAngleData dThetaX, dThetaY;

    dx1 = MouseSensorINTToFloat(dx1_i,1.0f);
    dy1 = MouseSensorINTToFloat(dy1_i,_cst_MOUSE_SENSOR_SCALE_FACTOR_Y);
    dx2 = MouseSensorINTToFloat(dx2_i,1.0f);
    dy2 = MouseSensorINTToFloat(dy2_i,_cst_MOUSE_SENSOR_SCALE_FACTOR_Y);
    dThetaX = dy2* (0.0f-_cst_ROTOR_RADIUS_RECIPROCAL);
    dThetaY = dy1* _cst_ROTOR_RADIUS_RECIPROCAL;

    return (dx1 - dx2) * (0.0f - _cst_ROTOR_RADIUS_RECIPROCAL) / (dThetaX - dThetaY);
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/


/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
