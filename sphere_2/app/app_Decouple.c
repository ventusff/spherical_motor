#include "app_Decouple.h"

/** 实现 由虚电流、动子姿态解算实际电流
 * 
 * */

extern RotorOrientation g_tRotorOrientation;
MainPID_Decouple_T l_tDecouple;

/** 函数：由 目前的动子姿态 以及 PID 给出的虚电流 解算出 实际电流
 * 输入：
 * 0.动子姿态: 目前定义为旋转矩阵. 在共享变量中包含，见所include 的 app_RotorRotate.h
 * 1.PID 给出的虚电流，维度为 3 或 6. 因维度不固定，因此为指针.维度定义：_cfg_MAIN_PID_CHANNEL_NUM
 * 输出(实际也是输入，指针定义)：
 * 2.解算出的实际电流 i1, i2, ..., i8. 维度定义：_cfg_SUB_PID_CHANNEL_NUM
 * 用到的配置信息：
 * 线圈组合方式.
 * **/
void app_Main_PID_Decouple(_unit_CURRENT_FLOAT_DATA* i_virtual, _unit_CURRENT_FLOAT_DATA* i_real)
{
    {
        //注意到：由于动子的变换没有平动自由度：即动子的位姿变换矩阵4*4可以简化成姿态变换矩阵[3*3]；
        //而[3*3]的旋转矩阵的逆就是本身的转置.

        //利用转置求动子旋转矩阵的逆，并将其应用到各线圈以求出各线圈在动子坐标系下的坐标.
        _unit_MAIN_PID_DATA pRotateInverseData[3*3];
        arm_matrix_instance_f32 pRotate, pRotateInverse;
        pRotate.numRows = 3;
        pRotate.numCols = 3;
        pRotate.pData = g_tRotorOrientation.R_rotation;
        pRotateInverse.numCols = 3;
        pRotateInverse.numRows = 3;
        pRotateInverse.pData = pRotateInverseData;
        arm_mat_trans_f32(&pRotate, &pRotateInverse);

        //利用变换矩阵逆阵求出线圈在动子坐标系下的坐标
        arm_matrix_instance_f32 pCoilInitPostion, pCoilNowPosition;//3*_cfg_COIL_NUM的矩阵
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

    //利用Coil_Txyz_Coeffs[]的16个线圈的系数 与 线圈的ROUGH GROUPING
    //逆向求解出Real Current
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

                        //这一步被取代了！若线圈或功放失效，直接设置相应direction为0即可。
                        //changed by ventus guo 20181011
                        /*
                        if(dac_number == 2)
                        {
                            //2线圈暂时失效
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
                //这一步被取代了。
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
                //这一步已经被取代了。
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

/** 求得对于一组线圈的确定位置(phi0,theta0)，其通单位电流对Tx,Ty,Tz的贡献系数
 * 输入：
 * 1.该线圈球坐标位置(phi0,theta0)
 * 输出
 * 2.该线圈通单位电流对Tx,Ty,Tz的贡献系数kTx,kTy,kTz
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

    //没有考虑电流面密度
    /*
    //归一化 e-9
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

    //考虑电流面密度：changed by ventus guo 20181013
    //归一化：同乘1e2：这样设定的1N·m力矩，相当于0.01N·m
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

/** 函数：解N元一次方程组
 * 
 * 
 * */
void SolveRealCurrentEquations(
        _unit_MAIN_PID_DATA _A[][_cfg_DAC_GROUP_NUM], _unit_CURRENT_FLOAT_DATA* x, const _unit_CURRENT_FLOAT_DATA* _b)
{
    #if !_cfg_LEVITATION_ON && _cfg_DEBOUPLE_USE_FORMULA
    {
        //利用A最后一列为1，先将方程化为3元1次方程组
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
        //依次标记每一列的主元所在行。使用此标记来免去方程交换操作。
        //有效的pivot_rows: [0] ~ [iter]
        uint8_t pivot_rows[_cfg_DECOUPLE_EQUATION_ROWS] = {0,0,0};
        //依次记录其他行的行号。使用此标记来免去方程交换操作。
        //有效的other_rows: [0] ~ [_cfg_DECOUPLE_EQUATION_ROWS - iter - 1]
        uint8_t other_rows[_cfg_DECOUPLE_EQUATION_ROWS] = {0,1,2};

        _unit_MAIN_PID_DATA A[_cfg_DECOUPLE_EQUATION_ROWS][_cfg_DECOUPLE_EQUATION_COLUMNS];
        //在第一次循环遍历中，就可以找到第一列的主元。
        _unit_MAIN_PID_DATA _current_pivot = _mth_abs(_A[0][0]); pivot_rows[0] = 0;
        for(uint8_t i = 0; i<_cfg_DECOUPLE_EQUATION_ROWS; i++)
        {
            //寻找第一列的主元
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

        //遍历所有，列主元高斯消元并得行阶梯
        for(uint8_t iter = 0; iter < _cfg_DECOUPLE_EQUATION_ROWS; iter++)
        {
            //找到新的主元
            //注意这里跳过了第一次，因为第一次提前找过了
            //如果不跳过，那么other_rows的初始值不能是0,0,0，而应该是0,1,2
            if(iter!=0)
            {
                //遍历other_rows
                pivot_rows[iter] = other_rows[0];
                _current_pivot = _mth_abs(A[other_rows[0]][iter]);

                if(iter == _cfg_DECOUPLE_EQUATION_ROWS - 1)
                {
                    //进入这里意味着other_rows有效元素只有一个
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

            //主元行后面的列除以主元
            for(uint8_t j = iter + 1; j < _cfg_DECOUPLE_EQUATION_COLUMNS; j++)
            {
                A[pivot_rows[iter]][j] /= A[pivot_rows[iter]][iter];
            }
            //主元行第iter列直接变为1
            A[pivot_rows[iter]][iter] = 1;

            //其他行将第iter列消为0；同时，记录其他行的行号，作为下次找主元的遍历
            uint8_t index_other_row = 0;
            for(uint8_t i = 0; i < _cfg_DECOUPLE_EQUATION_ROWS; i++)
            {
                //看i是否是这一次的pivot_rows
                if(i!=pivot_rows[iter])
                {
                    for(uint8_t j = iter + 1; j < _cfg_DECOUPLE_EQUATION_COLUMNS; j++)
                    {
                        A[i][j] -= A[pivot_rows[iter]][j]*A[i][iter];
                    }
                    A[i][iter] = 0;
                }

                //看i是否在之前出现过所有的（包括这次）pivot_rows中
                uint8_t idx;
                for(idx = 0; idx <= iter; idx++)
                {
                    if(i == pivot_rows[idx])
                    {
                        break;
                    }
                }
                //如果未出现过
                if(idx > iter)
                {
                    other_rows[index_other_row++] = i;
                }
            }
        }//end of 迭代

        //得出基底向量与特解
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
                //TODO: 对于大于一维的解空间，这里是要修正的
                basis_vector[idx] = 1;
                special_solution[idx] = 0;
            }
            sum_of_b_ += basis_vector[idx] * basis_vector[idx];
            sum_of_s_ += basis_vector[idx] * special_solution[idx];
        }

        //防止sum_of_b_ 为 0
        if(sum_of_b_ < 1e-3f)
        {
            //在这里使用补充加和的方式，而不是重新计算sum
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

        //求使电流平方和最小的t*
        _unit_CURRENT_FLOAT_DATA t_ = 0.0f - sum_of_s_/sum_of_b_;

        //求出最后的实电流
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

    //用[1,1,1]初始化k,由于力矩的归一化，
    _unit_MAIN_PID_DATA tmp[6] = {1,1,1,1,1,1};
    for(uint8_t i = 0; i< _cfg_MAIN_PID_CHANNEL_NUM; i++)
    {
        l_tDecouple.VirtualCurrentK[i] = tmp[i];
    }
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
