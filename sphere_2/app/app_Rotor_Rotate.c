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
        //�������װ�������ݴ�������ô�������ڲ����й���װ�ý��㣬ֱ�����ù���װ������
        if(g_tUSART_2_GetLine.ReadyFlag)
        {
            //safety && data complete check
            if(g_tUSART_2_GetLine.ReceiveCount == 0 && g_tUSART_2_GetLine.strLen == 16)
            {
                if(g_tUSART_2_GetLine.receive_buffer[0] == '^' && g_tUSART_2_GetLine.receive_buffer[13] == '^')
                {
                    //��Ҫ����������оƬ�����ۻ�����
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

                    // ��RPY�Ǽ������̬����
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

                    // ֱ�Ӹ���RPY��
                    g_tRotorOrientation.ThetaX = drx;
                    g_tRotorOrientation.ThetaY = dry;
                    g_tRotorOrientation.ThetaZ = drz;


                    g_tUSART_2_GetLine.ReadyFlag = 0;
                    return;
                }
            }
        }
    #endif

    //�������оƬ�������¶�����̬
    //��̬�����ڣ�1.PID���¼������ֵ 2.��������Ƴ�ʵ����(����app_Decouple.c)
    {
        g_tRotorOrientation._flag_update = 0;
        
        g_tRotorOrientation.DX1 += g_tADNS9800.MotionX_1;
        g_tRotorOrientation.DY1 += g_tADNS9800.MotionY_1;
        g_tRotorOrientation.DX2 += g_tADNS9800.MotionX_2;
        g_tRotorOrientation.DY2 += g_tADNS9800.MotionY_2;

        _unit_MouseSensorFLOATData drx,dry,drz;
		MouseSensorToDTheta(g_tRotorOrientation.DX1,g_tRotorOrientation.DY1,
			g_tRotorOrientation.DX2,g_tRotorOrientation.DY2,&drx,&dry,&drz);

        // ��ŷ���ǵó���ת����
        // ���������ֱ�Ӳ⵽�������ת��ʵ�ʾ�����������ϵ����ת
        // ��������£����������ʱ���ƣ�6*0.5+21*0.1 = 5.1 us  //��ʹ��ʹ��DSP��Ҳ��ʹ��Ӳ�����㣬���ƺ�ʱ�� 120 ~ 200us����
        // ����9*4λ8bit���ݴ�����ʱ��̹��ƣ� 2.5ms
        // ����3*4λ8bit���ݴ�����ʱ��̹��ƣ� 833.3us
        // ���Կ��������������ͨѶ�ĺ�ʱ�Ƿǳ��ٵġ�
        // ��˶����������ݵĸ��£������ڼ�����˽���ΪRPYֵ�����¼������̬���󲢸���

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

        //����ת���������
        //��
        _unit_MAIN_PID_DATA m1 = (g_tRotorOrientation.Ro_stator[7] - g_tRotorOrientation.Ro_stator[5]) * 0.5f,
            m2 = (g_tRotorOrientation.Ro_stator[2] - g_tRotorOrientation.Ro_stator[6]) * 0.5f,
            m3 = (g_tRotorOrientation.Ro_stator[3] - g_tRotorOrientation.Ro_stator[1]) * 0.5f; 
        _unit_MAIN_PID_DATA s = (m1*m1+m2*m2+m3*m3);
        //1e-5�����������һ���ۻ�
        //��DY = 1�������£�s = 1.7e-8
        //��DY = 3�������£�s = 1.6e-7
        //��DY = 9�������£�s = 1.4e-6
        //ʵ���п��Է��֣����оƬ���������ɴﵽDY = 3, s = 1.6e-7��������
        if(s<1e-3f)
        {
            //�����δ����������ת
            return;
        }
        //��������ˣ�������㲢�ҷ���һ�θ���
        g_tRotorOrientation.DX1 = 0;
        g_tRotorOrientation.DY1 = 0;
        g_tRotorOrientation.DX2 = 0;
        g_tRotorOrientation.DY2 = 0;
        g_tRotorOrientation._flag_update = 1;

        //issue: ���ٶȽ���
        //��Ϊ�����﷢������̬���������������оƬ���ݸ��µ�ǰ���ٶ�
        //  ���ǣ����оƬ�����ۻ������Ǿ���ÿһʱ�̵�������Ϊ�仹��׼ȷ��
        //  ��ʽ1��ֱ������ÿ�����оƬ�ĵ�����΢Ԫ���ݣ�ֱ�Ӽ�����ٶȣ�
        //      ֱ��д�����оƬ������
        //  ��ʽ2�����������ۻ�֮����Ϊ��Ч�Ķ������ۻ�΢Ԫ���ݣ�������ٶ�
        //      д������
        //  �ֲ��÷�ʽ1


        _unit_MAIN_PID_DATA sqrt_s;
        arm_sqrt_f32(s,&sqrt_s);
        _unit_MAIN_PID_DATA sqrt_s_reciprocal = 1.0f/sqrt_s;
        g_tRotorOrientation.Ro_axis_stator[0] = m1 * sqrt_s_reciprocal;
        g_tRotorOrientation.Ro_axis_stator[1] = m2 * sqrt_s_reciprocal;
        g_tRotorOrientation.Ro_axis_stator[2] = m3 * sqrt_s_reciprocal;
        //��
        _unit_MAIN_PID_DATA c_ = (g_tRotorOrientation.Ro_stator[0] + g_tRotorOrientation.Ro_stator[4] + g_tRotorOrientation.Ro_stator[8] - 1) * 0.5f;
        //���Կ�����������acosf	������ʱ���ڷǳ�С�ĽǶ��Ѿ��ۻ�����һ�������
        //e.g.: ��ʵ�Ƕ� = 0.019542, ʵ��ARM���� = 0.0218315758
        //�����е�cosֵ��1e-5��������acos�����3e-3���ҵ����
        /** �����п��Է��֣�
         * ���ڱȽ�С����ת�Ƕȣ����磺angle = 0.00088269 (rx = -0.00079999998, s �� 7e-7����)
         * ��acosf���Ź̶���3e-3���ҵ����Ѿ�Զ���ڽǶȴ�ʱ������8e-4��������
         *      ���۲��п��Կ���sin,cos������ת����ļ����������С�Ľ���û���ر��
         * ��������acosf����������Ҫ���ֽǶ���3e-2�ļ���----����ʵ���Ѿ����������ǵ�Ҫ����
         * ��ˣ�����������acosf��ֱ������sin������������ (��Ȼangleֵ��ʵ����ֱ��ʹ�õ�)
         *      �����ֹ۲쵽��c_��ֵ����ͻ��� 5e-5����������s = 1.5e-7������c_���������1e-8����
         * ��ˣ�Ŀǰ��ȡ�������ۻ����оƬ�仯ֵ��ֱ��s �ﵽ 1e-5������
         * 
         * ʵ��1��s�ۼƵ�1e-5�Ÿ��¡�
         *      ��Ȼ���֣�c_����5e-5������ʵ��1-c_ = 8.7e-6
         *      ��s_ Ҳ����4e-3��������
         * ʵ��2��s�ۼƵ�1e-4�Ÿ��¡��Ƕ�������0.012�����оƬ��ֵ������100
         *      s_ �ﵽ��0.012�����������2e-3�����Խ���;
         *      c_ ����������1e-5���ڣ�ʵ�����1e-4�����
         *      Ro_rotor �ձ����1e-3�������Ȼ��R_rotor��������1e-2~1e-3֮��
         *      ��ˣ����ǲ���
         * ʵ��3��s�ۼƵ�1e-3�Ÿ��¡��Ƕ�������0.033 ���оƬ��ֵ������200~300
         *      s �����1e-7����
         *      c_: 0.99947�����0.99942, 5e-5���
         *      s_: 0.03266, ���0.03381, 1e-3���
         *      Ro_rotor: ������1e-2������4e-4��������
         *      ���¿��Խ���
         * */
        //g_tRotorOrientation.Ro_angle = acosf( c_ );
        //_unit_MAIN_PID_DATA s_ = arm_sin_f32(g_tRotorOrientation.Ro_angle);
        _unit_MAIN_PID_DATA s_;
        arm_sqrt_f32((1.0f - c_ * c_), &s_);

        //�����ǰ���ڶ���ϵ�µ�����
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


        //���ݵ�ǰ���ڶ���ϵ�����꣬���Ƴ�����ϵ�µ���ת����
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

        //�Ӷ�����ǵ�������ת������������̬
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

    //���ݶ�����̬����ת�õõ���ת����
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
    //������̬��ּ������ǰ psi,theta,phi(��x����y����z)
    //��̬����ֱ�������ת�ǵġ�Ҫ�������������ת֮�������̬����������x,y,z
    //��������RPY��ϱ任��ʾ�˶������˶�ѧ�֮ⷨ��RPY�任�ⷨ
    //!!VENTUSFF ע��˴��еڶ��⣡
    //  ���Կ����ڳ���ʱ500Hz������Ƶ���£�һ�����ڽǶ�������仯��5~10������
    {
        g_tRotorOrientation.ThetaX_old = g_tRotorOrientation.ThetaX;
        g_tRotorOrientation.ThetaY_old = g_tRotorOrientation.ThetaY;
        g_tRotorOrientation.ThetaZ_old = g_tRotorOrientation.ThetaZ;

        //phi = ThetaZ = atan(ny,nx),phi = phi+180��ע��phi��������
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


/**�����������оƬ����������ת��Ϊ����������
 * ���룺���оƬ��������x, �������ӣ������ǵ��������ȫ��������µı��Σ�(Ĭ�ϣ�1.0)�����ò��� _cfg_ADNS9800CPI
 * ��������оƬ���������ݣ���λmm
 * */
_unit_MouseSensorFLOATData MouseSensorINTToFloat(_unit_MouseSensorINTData x, _unit_MouseSensorFLOATData ScaleFactor)
{
    //����
    static _unit_MouseSensorFLOATData CPI_to_mm = 25.4/_cfg_ADNS9800_CPI;
    //1 inch = 25.4mm���ʴ˵�λΪmm
    return ((_unit_MouseSensorFLOATData)x)*CPI_to_mm*ScaleFactor;
}

/**�����������оƬ��������dx1,dy1,dx2,dy2����Ϊdthetax,dthetay,dthetaz����ת��
 * ���룺
 * 1.dx1,dy1,dx2,dy2 ���оƬԭʼ��������
 * 2.dtx,dty,dtz �����������ת�ǣ���λ��rad
 * �����dThetaZ �����㷨�Ĳ�ֵ
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

    /* ������ʱ˵��
     * ��ΪARM��ϵ�ṹ������������������Ӳ����������ARM��ʵ�ֳ�����ʮ�ֺ�ʱ�ġ�
     * ARMָ���û��ֱ���ṩ�������ָ��������г��ֳ�������ʱ��ARM�����������C�⺯��
     * ���з��ϳ�������_rt_sdiv���޷��ϳ�������_rt_udiv������ʵ�ֳ���������
     * ���ݳ����ͱ������Ĳ�ͬ��32bit�ĳ�������һ��Ҫռ��20��140��ָ�����ڡ�
     * **/

    //*dThetaX = dy2/(0.0-_cst_ROTOR_RADIUS);//������ʱ�ϳ��������˷�
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

/**�����������оƬ��������dx1,dy1,dx2,dy2�����cos(theta2)
 * ���룺dx1,dy1,dx2,dy2 ���оƬԭʼ��������
 * �����cos(theta2) �������
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
