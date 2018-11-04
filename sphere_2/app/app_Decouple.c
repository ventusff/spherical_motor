#include "app_Decouple.h"

/** ʵ�� ���������������̬����ʵ�ʵ���
 * 
 * */

extern RotorOrientation g_tRotorOrientation;
MainPID_Decouple_T l_tDecouple;

/** �������� Ŀǰ�Ķ�����̬ �Լ� PID ����������� ����� ʵ�ʵ���
 * ���룺
 * 0.������̬: Ŀǰ����Ϊ��ת����. �ڹ�������а���������include �� app_RotorRotate.h
 * 1.PID �������������ά��Ϊ 3 �� 6. ��ά�Ȳ��̶������Ϊָ��.ά�ȶ��壺_cfg_MAIN_PID_CHANNEL_NUM
 * ���(ʵ��Ҳ�����룬ָ�붨��)��
 * 2.�������ʵ�ʵ��� i1, i2, ..., i8. ά�ȶ��壺_cfg_SUB_PID_CHANNEL_NUM
 * �õ���������Ϣ��
 * ��Ȧ��Ϸ�ʽ.
 * **/
void app_Main_PID_Decouple(_unit_CURRENT_FLOAT_DATA* i_virtual, _unit_CURRENT_FLOAT_DATA* i_real)
{
    {
        //ע�⵽�����ڶ��ӵı任û��ƽ�����ɶȣ������ӵ�λ�˱任����4*4���Լ򻯳���̬�任����[3*3]��
        //��[3*3]����ת���������Ǳ����ת��.

        //����ת��������ת������棬������Ӧ�õ�����Ȧ���������Ȧ�ڶ�������ϵ�µ�����.
        _unit_MAIN_PID_DATA pRotateInverseData[3*3];
        arm_matrix_instance_f32 pRotate, pRotateInverse;
        pRotate.numRows = 3;
        pRotate.numCols = 3;
        pRotate.pData = g_tRotorOrientation.R_rotation;
        pRotateInverse.numCols = 3;
        pRotateInverse.numRows = 3;
        pRotateInverse.pData = pRotateInverseData;
        arm_mat_trans_f32(&pRotate, &pRotateInverse);

        //���ñ任�������������Ȧ�ڶ�������ϵ�µ�����
        arm_matrix_instance_f32 pCoilInitPostion, pCoilNowPosition;//3*_cfg_COIL_NUM�ľ���
        pCoilInitPostion.numCols = _cfg_COIL_NUM;
        pCoilInitPostion.numRows = 3;
        pCoilInitPostion.pData = l_tDecouple.Coil_Init_Position;

        pCoilNowPosition.numCols = _cfg_COIL_NUM;
        pCoilNowPosition.numRows = 3;
        pCoilNowPosition.pData = l_tDecouple.Coil_Now_Position;
        arm_mat_mult_f32(&pRotateInverse, &pCoilInitPostion, &pCoilNowPosition);

        for(uint8_t i = 0; i<_cfg_COIL_NUM; i++)
        {
            Get1CoilTxyzCoef(l_tDecouple.Coil_Now_Position[i],
                l_tDecouple.Coil_Now_Position[i + _cfg_COIL_NUM], 
                l_tDecouple.Coil_Now_Position[i + 2*_cfg_COIL_NUM],
                &l_tDecouple.Coil_Txyz_Coeffs[i],
                &l_tDecouple.Coil_Txyz_Coeffs[i + _cfg_COIL_NUM],
                &l_tDecouple.Coil_Txyz_Coeffs[i + 2*_cfg_COIL_NUM]);
        }
    }

    //����Coil_Txyz_Coeffs[]��16����Ȧ��ϵ�� �� ��Ȧ��ROUGH GROUPING
    //��������Real Current
    {
        if(_cfg_MAIN_PID_CHANNEL_NUM > _cfg_DAC_GROUP_NUM)
        {
            printx("_cfg_DAC_GROUP_NUM needs to be equal or greater than _cfg_MAIN_PID_CHANNEL_NUM\n");
            while(1);
        }
        for(uint8_t i = 0; i < _cfg_DAC_GROUP_NUM; i++)
        {
            if(i < _cfg_MAIN_PID_CHANNEL_NUM)
            {
                for(uint8_t j = 0; j< _cfg_DAC_GROUP_NUM; j++)
                {
                    l_tDecouple.matrix_Real_To_Virtual[i][j] = 0;
                    for(uint8_t k = 0; k< _DAC_GROUP_X_NUM[j]; k++)
                    {
                        uint8_t dac_number = _cfg_Group_of_DAC[j][k];

                        //��һ����ȡ���ˣ�����Ȧ�򹦷�ʧЧ��ֱ��������ӦdirectionΪ0���ɡ�
                        //changed by ventus guo 20181011
                        /*
                        if(dac_number == 2)
                        {
                            //2��Ȧ��ʱʧЧ
                        }
                        else{
                        */
                            l_tDecouple.matrix_Real_To_Virtual[i][j] +=
                                (l_tDecouple.Coil_Txyz_Coeffs[_cfg_DAC_number_To_coil_number[dac_number-1][0] - 1 + i*_cfg_COIL_NUM]
                                    * _cfg_Directions[dac_number-1][0]
                                + l_tDecouple.Coil_Txyz_Coeffs[_cfg_DAC_number_To_coil_number[dac_number-1][1] - 1 + i*_cfg_COIL_NUM]
                                    * _cfg_Directions[dac_number-1][1]);
                        /*
                        }
                        */

                    }
                }
            }
            else{
                //��һ����ȡ���ˡ�
                //changed by ventus 20181007
                /*
                for(uint8_t j = 0; j<_cfg_DAC_GROUP_NUM; j++)
                {
                    l_tDecouple.matrix_Real_To_Virtual[i][j] = 1;
                }
                */
            }
        }
        _unit_CURRENT_FLOAT_DATA i_CoilGroup[_cfg_DAC_GROUP_NUM];
        _unit_CURRENT_FLOAT_DATA i_virtual_expand[_cfg_DAC_GROUP_NUM];
        for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM; i++)
        {
            if(i< _cfg_MAIN_PID_CHANNEL_NUM)
            {
                i_virtual_expand[i] = i_virtual[i];
            }
            else{
                //��һ���Ѿ���ȡ���ˡ�
                //changed by ventus 20181007
                /*
                i_virtual_expand[i] = 0.0f;
                */
            }
        }
        
        SolveRealCurrentEquations(l_tDecouple.matrix_Real_To_Virtual,i_CoilGroup,i_virtual_expand);
        for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM; i++)
        {
            for(uint8_t j = 0; j<_DAC_GROUP_X_NUM[i]; j++)
            {
                i_real[_cfg_Group_of_DAC[i][j]-1] = i_CoilGroup[i];
            }
        }
    }
}

/** ��ö���һ����Ȧ��ȷ��λ��(phi0,theta0)����ͨ��λ������Tx,Ty,Tz�Ĺ���ϵ��
 * ���룺
 * 1.����Ȧ������λ��(phi0,theta0)
 * ���
 * 2.����Ȧͨ��λ������Tx,Ty,Tz�Ĺ���ϵ��kTx,kTy,kTz
 * */
void Get1CoilTxyzCoef(_unit_MAIN_PID_DATA x, _unit_MAIN_PID_DATA y, _unit_MAIN_PID_DATA z,
    _unit_MAIN_PID_DATA* kTx, _unit_MAIN_PID_DATA* kTy, _unit_MAIN_PID_DATA* kTz)
{
    _unit_RotateAngleData phi0 = atan2f(y,x), theta0 = acosf(z/_cst_COIL_COOR_RADIUS);

    _unit_MAIN_PID_DATA sph_1,sph_3,cph_1,cph_3,sth_1,sth_3,sth_4,sth_5,cth_1,cth_2,cth_3,cth_4,cth_5;
    sph_1 = arm_sin_f32(phi0);
    cph_1 = arm_cos_f32(phi0);
    sth_1 = arm_sin_f32(theta0);
    cth_1 = arm_cos_f32(theta0);
    sph_3 = sph_1 * sph_1 * sph_1;
    cph_3 = cph_1 * cph_1 * cph_1;
    sth_3 = sth_1 * sth_1 * sth_1;
    sth_4 = sth_3 * sth_1;
    sth_5 = sth_4 * sth_1;
    cth_2 = cth_1 * cth_1;
    cth_3 = cth_2 * cth_1;
    cth_4 = cth_3 * cth_1;
    cth_5 = cth_4 * cth_1;

    //û�п��ǵ������ܶ�
    /*
    //��һ�� e-9
    *kTx = 7.397495f*sph_1*sth_1 
        - 3.698747f*sph_1*sth_3 
        - 2.755846f*sph_1*sth_5 
        - 18.42088f*sph_3*sth_3 
        + 16.53507f*sph_3*sth_5;
    *kTy = 0.942902f*cph_1*sth_1
        - 1.885804f*cph_3*sth_1
        + 9.210439f*cph_1*cth_2*sth_1
        - 2.755846f*cph_1*cth_4*sth_1
        - 14.64927f*cph_3*cth_2*sth_1
        + 16.53507f*cph_3*cth_4*sth_1;
    *kTz = 11.37598f*cph_1*cth_5*sph_1 
        - 7.956979f*cph_1*cth_3*sph_1 
        - 3.419005f*cph_1*cth_1*sph_1 
        - 0.3526024f*cph_1*cth_1*sph_1*sth_4;
    */

    //���ǵ������ܶȣ�changed by ventus guo 20181013
    //��һ����ͬ��1e2�������趨��1N��m���أ��൱��0.01N��m
    *kTx = 5.548121f*sph_1*sth_1 
        - 2.774061f*sph_1*sth_3 
        - 2.066884f*sph_1*sth_5 
        - 13.81566f*sph_3*sth_3 
        + 12.4013f*sph_3*sth_5;
    *kTy = 0.7071765f*cph_1*sth_1
        - 1.414353f*cph_3*sth_1
        + 6.907829f*cph_1*cth_2*sth_1
        - 2.066884f*cph_1*cth_4*sth_1
        - 10.98695f*cph_3*cth_2*sth_1
        + 12.4013f*cph_3*cth_4*sth_1;
    *kTz = 8.531988f*cph_1*cth_5*sph_1 
        - 5.967734f*cph_1*cth_3*sph_1 
        - 2.564254f*cph_1*cth_1*sph_1 
        - 0.2644518f*cph_1*cth_1*sph_1*sth_4;
}

/** ��������NԪһ�η�����
 * 
 * 
 * */
void SolveRealCurrentEquations(
        _unit_MAIN_PID_DATA _A[][_cfg_DAC_GROUP_NUM], _unit_CURRENT_FLOAT_DATA* x, const _unit_CURRENT_FLOAT_DATA* _b)
{
    #if !_cfg_LEVITATION_ON && _cfg_DEBOUPLE_USE_FORMULA
    {
        //����A���һ��Ϊ1���Ƚ����̻�Ϊ3Ԫ1�η�����
        _unit_MAIN_PID_DATA A[_cfg_DAC_GROUP_NUM-1][_cfg_DAC_GROUP_NUM-1];
        _unit_CURRENT_FLOAT_DATA b[_cfg_DAC_GROUP_NUM-1];
        for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM-1; i++)
        {
            b[i] = _b[i] - _A[i][0];
            for(uint8_t j = 0; j<_cfg_DAC_GROUP_NUM-1; j++)
            {
                A[i][j] = _A[i][j+1] - _A[i][0];
            }
        }
        _unit_MAIN_PID_DATA D = detOf3x3(A[0][0],A[0][1],A[0][2],A[1][0],A[1][1],A[1][2],A[2][0],A[2][1],A[2][2]);
        _unit_MAIN_PID_DATA X = detOf3x3(b[0],A[0][1],A[0][2],b[1],A[1][1],A[1][2],b[2],A[2][1],A[2][2]);
        _unit_MAIN_PID_DATA Y = detOf3x3(A[0][0],b[0],A[0][2],A[1][0],b[1],A[1][2],A[2][0],b[2],A[2][2]);
        _unit_MAIN_PID_DATA Z = detOf3x3(A[0][0],A[0][1],b[0],A[1][0],A[1][1],b[1],A[2][0],A[2][1],b[2]);
        x[0] = X/D;
        x[1] = Y/D;
        x[2] = Z/D;
        x[3] = 1.0f - x[0] - x[1] - x[2];
    }
    #else
    {
        //���α��ÿһ�е���Ԫ�����С�ʹ�ô˱������ȥ���̽���������
        //��Ч��pivot_rows: [0] ~ [iter]
        uint8_t pivot_rows[_cfg_DECOUPLE_EQUATION_ROWS] = {0,0,0};
        //���μ�¼�����е��кš�ʹ�ô˱������ȥ���̽���������
        //��Ч��other_rows: [0] ~ [_cfg_DECOUPLE_EQUATION_ROWS - iter - 1]
        uint8_t other_rows[_cfg_DECOUPLE_EQUATION_ROWS] = {0,1,2};

        _unit_MAIN_PID_DATA A[_cfg_DECOUPLE_EQUATION_ROWS][_cfg_DECOUPLE_EQUATION_COLUMNS];
        //�ڵ�һ��ѭ�������У��Ϳ����ҵ���һ�е���Ԫ��
        _unit_MAIN_PID_DATA _current_pivot = _mth_abs(_A[0][0]); pivot_rows[0] = 0;
        for(uint8_t i = 0; i<_cfg_DECOUPLE_EQUATION_ROWS; i++)
        {
            //Ѱ�ҵ�һ�е���Ԫ
            if(i!=0)
            {
                if(_mth_abs(_A[i][0]) > _current_pivot)
                {
                    _current_pivot = _mth_abs(_A[i][0]);
                    pivot_rows[0] = i;
                }
            }

            for(uint8_t j = 0; j < _cfg_DECOUPLE_EQUATION_COLUMNS - 1; j++)
            {
                A[i][j] = _A[i][j];
            }
            A[i][_cfg_DECOUPLE_EQUATION_COLUMNS - 1] = _b[i];
        }

        //�������У�����Ԫ��˹��Ԫ�����н���
        for(uint8_t iter = 0; iter < _cfg_DECOUPLE_EQUATION_ROWS; iter++)
        {
            //�ҵ��µ���Ԫ
            //ע�����������˵�һ�Σ���Ϊ��һ����ǰ�ҹ���
            //�������������ôother_rows�ĳ�ʼֵ������0,0,0����Ӧ����0,1,2
            if(iter!=0)
            {
                //����other_rows
                pivot_rows[iter] = other_rows[0];
                _current_pivot = _mth_abs(A[other_rows[0]][iter]);

                if(iter == _cfg_DECOUPLE_EQUATION_ROWS - 1)
                {
                    //����������ζ��other_rows��ЧԪ��ֻ��һ��
                }
                else{
                    for(uint8_t idx = 1; idx < _cfg_DECOUPLE_EQUATION_ROWS - iter; idx++)
                    {
                        if(_mth_abs(A[other_rows[idx]][iter]) > _current_pivot)
                        {
                            pivot_rows[iter] = other_rows[idx];
                            _current_pivot = _mth_abs(A[other_rows[idx]][iter]);
                        }
                    }
                }

            }

            //��Ԫ�к�����г�����Ԫ
            for(uint8_t j = iter + 1; j < _cfg_DECOUPLE_EQUATION_COLUMNS; j++)
            {
                A[pivot_rows[iter]][j] /= A[pivot_rows[iter]][iter];
            }
            //��Ԫ�е�iter��ֱ�ӱ�Ϊ1
            A[pivot_rows[iter]][iter] = 1;

            //�����н���iter����Ϊ0��ͬʱ����¼�����е��кţ���Ϊ�´�����Ԫ�ı���
            uint8_t index_other_row = 0;
            for(uint8_t i = 0; i < _cfg_DECOUPLE_EQUATION_ROWS; i++)
            {
                //��i�Ƿ�����һ�ε�pivot_rows
                if(i!=pivot_rows[iter])
                {
                    for(uint8_t j = iter + 1; j < _cfg_DECOUPLE_EQUATION_COLUMNS; j++)
                    {
                        A[i][j] -= A[pivot_rows[iter]][j]*A[i][iter];
                    }
                    A[i][iter] = 0;
                }

                //��i�Ƿ���֮ǰ���ֹ����еģ�������Σ�pivot_rows��
                uint8_t idx;
                for(idx = 0; idx <= iter; idx++)
                {
                    if(i == pivot_rows[idx])
                    {
                        break;
                    }
                }
                //���δ���ֹ�
                if(idx > iter)
                {
                    other_rows[index_other_row++] = i;
                }
            }
        }//end of ����

        //�ó������������ؽ�
        _unit_CURRENT_FLOAT_DATA basis_vector[_cfg_DAC_GROUP_NUM] = {0,0,0,0};
        _unit_CURRENT_FLOAT_DATA special_solution[_cfg_DAC_GROUP_NUM] = {0,0,0,0};
        _unit_CURRENT_FLOAT_DATA sum_of_b_ = 0;
        _unit_CURRENT_FLOAT_DATA sum_of_s_ = 0;

        for(uint8_t idx = 0; idx < _cfg_DAC_GROUP_NUM; idx++)
        {
            if(idx < _cfg_MAIN_PID_CHANNEL_NUM)
            {
                basis_vector[idx] = 0.0f - A[pivot_rows[idx]][_cfg_MAIN_PID_CHANNEL_NUM];
                special_solution[idx] = A[pivot_rows[idx]][_cfg_DECOUPLE_EQUATION_COLUMNS - 1];
            }
            else{
                //TODO: ���ڴ���һά�Ľ�ռ䣬������Ҫ������
                basis_vector[idx] = 1;
                special_solution[idx] = 0;
            }
            sum_of_b_ += basis_vector[idx] * basis_vector[idx];
            sum_of_s_ += basis_vector[idx] * special_solution[idx];
        }

        //��ֹsum_of_b_ Ϊ 0
        if(sum_of_b_ < 1e-3f)
        {
            //������ʹ�ò���Ӻ͵ķ�ʽ�����������¼���sum
            //sum_of_b_ = 0;
            //sum_of_s_ = 0;

            uint8_t max_num_ = 0;
            _unit_CURRENT_FLOAT_DATA max_b_2_ = basis_vector[0] * basis_vector[0];
            for(uint8_t idx = 1; idx < _cfg_DAC_GROUP_NUM; idx++)
            {
                _unit_CURRENT_FLOAT_DATA tmp = basis_vector[idx] * basis_vector[idx];
                if(tmp > max_b_2_)
                {
                    max_b_2_ = tmp;
                    max_num_ = idx;
                }
            }

            sum_of_b_ += max_b_2_;
            sum_of_s_ += basis_vector[max_num_] * special_solution[max_num_];
        }

        //��ʹ����ƽ������С��t*
        _unit_CURRENT_FLOAT_DATA t_ = 0.0f - sum_of_s_/sum_of_b_;

        //�������ʵ����
        for(uint8_t idx = 0; idx < _cfg_DAC_GROUP_NUM; idx++)
        {
            x[idx] = basis_vector[idx] * t_ + special_solution[idx];
        }

    }
    #endif
}

_unit_MAIN_PID_DATA detOf3x3(
    _unit_MAIN_PID_DATA a11,_unit_MAIN_PID_DATA a12,_unit_MAIN_PID_DATA a13,
    _unit_MAIN_PID_DATA a21,_unit_MAIN_PID_DATA a22,_unit_MAIN_PID_DATA a23,
    _unit_MAIN_PID_DATA a31,_unit_MAIN_PID_DATA a32,_unit_MAIN_PID_DATA a33)
{
    return a11*(a22*a33-a23*a32)-a12*(a21*a33-a23*a31)+a13*(a21*a32-a22*a31);
}

void app_Main_PID_Decouple_Initializes(void)
{
    for(uint16_t i = 0; i< 3*_cfg_COIL_NUM; i++)
    {
        l_tDecouple.Coil_Now_Position[i] = 0;
        l_tDecouple.Coil_Init_Position[i] = _cst_COIL_POSITION[i];
    }

    for(uint16_t i = 0; i< _cfg_MAIN_PID_CHANNEL_NUM * _cfg_COIL_NUM; i++)
    {
        l_tDecouple.Coil_Txyz_Coeffs[i] = 0;
    }

    for(uint8_t i = 0; i< _cfg_DAC_GROUP_NUM; i++)
    {
			for(uint8_t j = 0; j< _cfg_DAC_GROUP_NUM; j++)
        l_tDecouple.matrix_Real_To_Virtual[i][j] = 0;
    }

    //��[1,1,1]��ʼ��k,�������صĹ�һ����
    _unit_MAIN_PID_DATA tmp[6] = {1,1,1,1,1,1};
    for(uint8_t i = 0; i< _cfg_MAIN_PID_CHANNEL_NUM; i++)
    {
        l_tDecouple.VirtualCurrentK[i] = tmp[i];
    }
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
