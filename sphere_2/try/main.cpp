#include <iostream>
#include <math.h>
using namespace std;
#define _mth_abs(x) (x>0?x:(0.0f-x))
#define _cfg_DECOUPLE_EQUATION_ROWS 3
#define _cfg_DECOUPLE_EQUATION_COLUMNS 5
#define _cfg_DAC_GROUP_NUM 4
#define _cfg_MAIN_PID_CHANNEL_NUM 3
#define _cfg_COIL_NUM 16
#define _cfg_DAC_CHANNEL_NUM 8
#define _cst_COIL_COOR_RADIUS 0.037f

//这个一定要酌情添加！！！！！！！
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;


const uint8_t _DAC_GROUP_X_NUM[_cfg_DAC_GROUP_NUM] = {2,2,2,2};

//功放的分组（注意!不是线圈的分组）
const uint8_t _cfg_Group_of_DAC[_cfg_DAC_GROUP_NUM][2] = {{1,5},{2,7},{3,6},{4,8}};

//功放编号 与 线圈的对应关系（以及电流方向）
const uint8_t _cfg_DAC_number_To_coil_number[_cfg_DAC_CHANNEL_NUM][2] = 
//功放：1,     2,     3,      4,       5,      6,          7,       8
    {{1,5}, {2,6},  {3,7},  {4,8},  {9,10}, {11,12},    {13,14},{15,16}};

//与上面线圈编号一一对应的电流方向
const int8_t _cfg_Directions[_cfg_DAC_CHANNEL_NUM][2] =                 
    {{1,-1},{0,0},  {1,-1}, {1,-1}, {1,1},  {1,1},      {1,1},  {1,1}};

const float _cst_COIL_POSITION[3*_cfg_COIL_NUM] = 
{
    0.037,0.02616295,0.0,-0.02616295,-0.037,-0.02616295,0.0,0.02616295,0.03021037,0.02136196,0.0,-0.02136196,-0.03021037,-0.02136196,0.0,0.02136196,
    0.0,0.02616295,0.037,0.02616295,0.0,-0.02616295,-0.037,-0.02616295,0.0,0.02136196,0.03021037,0.02136196,0.0,-0.02136196,-0.03021037,-0.02136196,
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.02136196,-0.02136196,-0.02136196,-0.02136196,-0.02136196,-0.02136196,-0.02136196,-0.02136196
};

typedef struct{
    float Coil_Init_Position[3*_cfg_COIL_NUM];  //线圈初始位置.
    float Coil_Now_Position[3*_cfg_COIL_NUM];  //旋转后，根据动子新姿态，在动子坐标系下的线圈位置
    float VirtualCurrentK[_cfg_MAIN_PID_CHANNEL_NUM];
    float Coil_Txyz_Coeffs[_cfg_MAIN_PID_CHANNEL_NUM * _cfg_COIL_NUM];
    float matrix_Real_To_Virtual[_cfg_DAC_GROUP_NUM][_cfg_DAC_GROUP_NUM];
} MainPID_Decouple_T;

MainPID_Decouple_T l_tDecouple;



void printA(float A[_cfg_DECOUPLE_EQUATION_ROWS][_cfg_DECOUPLE_EQUATION_COLUMNS])
{
    cout <<endl;
    cout<<"A: " <<endl;
    for(uint8_t i = 0; i< _cfg_DECOUPLE_EQUATION_ROWS; i++)
    {
        for(uint8_t j = 0; j< _cfg_DECOUPLE_EQUATION_COLUMNS; j++)
        {
            cout<<A[i][j] <<", ";
        }
        cout <<endl;
    }
}

void Get1CoilTxyzCoef(float x, float y, float z,
    float* kTx, float* kTy, float* kTz)
{
    float phi0 = atan2(y,x), theta0 = acos(z/_cst_COIL_COOR_RADIUS);

    float sph_1,sph_3,cph_1,cph_3,sth_1,sth_3,sth_4,sth_5,cth_1,cth_2,cth_3,cth_4,cth_5;
    sph_1 = sin(phi0);
    cph_1 = cos(phi0);
    sth_1 = sin(theta0);
    cth_1 = cos(theta0);
    sph_3 = sph_1 * sph_1 * sph_1;
    cph_3 = cph_1 * cph_1 * cph_1;
    sth_3 = sth_1 * sth_1 * sth_1;
    sth_4 = sth_3 * sth_1;
    sth_5 = sth_4 * sth_1;
    cth_2 = cth_1 * cth_1;
    cth_3 = cth_2 * cth_1;
    cth_4 = cth_3 * cth_1;
    cth_5 = cth_4 * cth_1;

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
    float tmp[6] = {1,1,1,1,1,1};
    for(uint8_t i = 0; i< _cfg_MAIN_PID_CHANNEL_NUM; i++)
    {
        l_tDecouple.VirtualCurrentK[i] = tmp[i];
    }
}

void matrix_t(float *a_matrix, const float *b_matrix, int krow, int kline)
////////////////////////////////////////////////////////////////////////////
//	a_matrix:转置后的矩阵
//	b_matrix:转置前的矩阵
//	krow    :行数
//	kline   :列数
////////////////////////////////////////////////////////////////////////////
{
	int k, k2;   
 
	for (k = 0; k < krow; k++)
	{
		for(k2 = 0; k2 < kline; k2++)
		{
			//a_matrix[k2][k] = b_matrix[k][k2];
            a_matrix[k+k2*krow] = b_matrix[k2+k*kline];
		}
	}
}

void matrix_m(float *a_matrix, const float *b_matrix, const float *c_matrix,
				int krow, int kline, int kmiddle, int ktrl)
////////////////////////////////////////////////////////////////////////////
//	a_matrix=b_matrix*c_matrix
//	krow  :行数
//	kline :列数
//	ktrl  :	大于0:两个正数矩阵相乘 不大于0:正数矩阵乘以负数矩阵
////////////////////////////////////////////////////////////////////////////
{
	int k, k2, k4;
	float stmp;
 
	for (k = 0; k < krow; k++)     
	{
		for (k2 = 0; k2 < kline; k2++)   
		{
			stmp = 0.0;
			for (k4 = 0; k4 < kmiddle; k4++)  
			{
                //stmp += b_matrix[k][k4] * c_matrix[k4][k2];
				stmp += b_matrix[k4+k*kmiddle] * c_matrix[k2+k4*kline];
			}
            //a_matrix[k][k2] = stmp;   
			a_matrix[k2+k*kline] = stmp;
		}
	}
	if (ktrl <= 0)   
	{
		for (k = 0; k < krow; k++)
		{
			for (k2 = 0; k2 < kline; k2++)
			{
				a_matrix[k2+k*kline] = -a_matrix[k2+k*kline];
			}
		}
	}
}

int main()
{
    app_Main_PID_Decouple_Initializes();
    float drx = 30.0f/180.0f*M_PI, dry = 0.3f, drz = 0.2f;
    float _b[_cfg_DECOUPLE_EQUATION_ROWS] = {0,1,0};
    
    float pDataCurrentRotate[3*3],
        sindrx = sin(drx), cosdrx = cos(drx),
        sindry = sin(dry), cosdry = cos(dry),
        sindrz = sin(drz), cosdrz = cos(drz);
    pDataCurrentRotate[0] = cosdry * cosdrz;
    pDataCurrentRotate[1] = cosdrz * sindrx * sindry - cosdrx * sindrz;
    pDataCurrentRotate[2] = sindrx * sindrz + cosdrx * cosdrz * sindry;
    pDataCurrentRotate[3] = cosdry * sindrz;
    pDataCurrentRotate[4] = cosdrx * cosdrz + sindrx * sindry * sindrz;
    pDataCurrentRotate[5] = cosdrx * sindry * sindrz - cosdrz * sindrx;
    pDataCurrentRotate[6] = 0.0f - sindry;
    pDataCurrentRotate[7] = cosdry * sindrx;
    pDataCurrentRotate[8] = cosdrx * cosdry;

{//FF
    cout <<endl;
    cout <<"rotor orientation: " <<endl;
    for(uint8_t i =0; i<3;i++)
    {
        for(uint8_t j =0;j<3;j++)
        {
            cout <<pDataCurrentRotate[j + i*3] <<", ";
        }
        cout <<endl;
    }
}



    //注意到：由于动子的变换没有平动自由度：即动子的位姿变换矩阵4*4可以简化成姿态变换矩阵[3*3]；
    //而[3*3]的旋转矩阵的逆就是本身的转置.

    //利用转置求动子旋转矩阵的逆，并将其应用到各线圈以求出各线圈在动子坐标系下的坐标.
    float pRotateInverseData[3*3];
    matrix_t(pRotateInverseData, pDataCurrentRotate,3,3);

{//FF
    cout <<endl;
    cout <<"stator orientation: " <<endl;
    for(uint8_t i =0; i<3;i++)
    {
        for(uint8_t j =0;j<3;j++)
        {
            cout <<pRotateInverseData[j + i*3] <<", ";
        }
        cout <<endl;
    }
}


    //利用变换矩阵逆阵求出线圈在动子坐标系下的坐标
    matrix_m(l_tDecouple.Coil_Now_Position, pRotateInverseData, l_tDecouple.Coil_Init_Position, 3,16,3,1);

    cout <<endl;
    cout <<"Coil_Now_Position: " <<endl;
    for(uint8_t j = 0; j<_cfg_COIL_NUM; j++)
    {
        cout <<"[" <<(int)j <<"]: ";
        for(uint8_t i = 0; i<3; i++)
        {
            cout << l_tDecouple.Coil_Now_Position[j + i*_cfg_COIL_NUM] <<", ";
        }
        cout <<endl;
    }

    for(uint8_t i = 0; i<_cfg_COIL_NUM; i++)
    {
        Get1CoilTxyzCoef(l_tDecouple.Coil_Now_Position[i],
            l_tDecouple.Coil_Now_Position[i + _cfg_COIL_NUM], 
            l_tDecouple.Coil_Now_Position[i + 2*_cfg_COIL_NUM],
            &l_tDecouple.Coil_Txyz_Coeffs[i],
            &l_tDecouple.Coil_Txyz_Coeffs[i + _cfg_COIL_NUM],
            &l_tDecouple.Coil_Txyz_Coeffs[i + 2*_cfg_COIL_NUM]);
    }


    //利用Coil_Txyz_Coeffs[]的16个线圈的系数 与 线圈的ROUGH GROUPING
    //逆向求解出Real Current
    {
        if(_cfg_MAIN_PID_CHANNEL_NUM > _cfg_DAC_GROUP_NUM)
        {
            cout <<"_cfg_DAC_GROUP_NUM needs to be equal or greater than _cfg_MAIN_PID_CHANNEL_NUM" <<endl;
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
                        /*
                        if(dac_number == 2)
                        {
                            //2线圈暂时失效
                        }
                        */
                        //else{
                            {//FF
                                cout <<"[" <<(int)i <<"][" <<(int)j <<"] = [" 
                                    <<(int)i <<"][" <<(int)(_cfg_DAC_number_To_coil_number[dac_number-1][0] - 1) <<"] + ["
                                    <<(int)i <<"][" <<(int)(_cfg_DAC_number_To_coil_number[dac_number-1][1] - 1) <<"], ";         
                            }

                            
                            l_tDecouple.matrix_Real_To_Virtual[i][j] +=
                                (l_tDecouple.Coil_Txyz_Coeffs[_cfg_DAC_number_To_coil_number[dac_number-1][0] - 1 + i*_cfg_COIL_NUM]
                                    * _cfg_Directions[dac_number-1][0]
                                + l_tDecouple.Coil_Txyz_Coeffs[_cfg_DAC_number_To_coil_number[dac_number-1][1] - 1 + i*_cfg_COIL_NUM]
                                    * _cfg_Directions[dac_number-1][1]);

                        //}

                    }

                    //FF
                    cout <<endl;
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

        cout <<endl;
        cout <<"Coil_Txyz_Coeffs" <<endl;
        for(uint8_t j = 0; j<_cfg_COIL_NUM; j++)
        {
            cout <<"[" <<(int)j <<"]: ";
            for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
            {
                cout <<l_tDecouple.Coil_Txyz_Coeffs[j+i*_cfg_COIL_NUM] <<",";
            }
            cout <<endl;
        }

        cout <<endl;
        cout <<"matrix_Real_To_Virtual: " <<endl;
        for(uint8_t i = 0; i<_cfg_MAIN_PID_CHANNEL_NUM; i++)
        {
            for(uint8_t j = 0; j<_cfg_DAC_GROUP_NUM; j++)
            {
                cout << l_tDecouple.matrix_Real_To_Virtual[i][j] <<", ";
            }
            cout <<endl;
        }



        //float _A[_cfg_DECOUPLE_EQUATION_ROWS][_cfg_DECOUPLE_EQUATION_COLUMNS - 1] = {{1,1,1,2},{3,3,5,6},{9,10,11,12}};
        //float _b[_cfg_DECOUPLE_EQUATION_ROWS] = {0,1,0};
        float x[4] = {0,0,0,0};//FF
        {
            //依次标记每一列的主元所在行。使用此标记来免去方程交换操作。
            uint8_t pivot_rows[_cfg_DECOUPLE_EQUATION_ROWS] = {0,0,0};
            //依次记录其他行的行号。使用此标记来免去方程交换操作。
            uint8_t other_rows[_cfg_DECOUPLE_EQUATION_ROWS] = {0,1,2};

            float A[_cfg_DECOUPLE_EQUATION_ROWS][_cfg_DECOUPLE_EQUATION_COLUMNS];
            //在第一次循环遍历中，就可以找到第一列的主元。
            float _current_pivot = _mth_abs(l_tDecouple.matrix_Real_To_Virtual[0][0]); pivot_rows[0] = 0;
            for(uint8_t i = 0; i<_cfg_DECOUPLE_EQUATION_ROWS; i++)
            {
                //寻找第一列的主元
                if(i!=0)
                {
                    if(_mth_abs(l_tDecouple.matrix_Real_To_Virtual[i][0]) > _current_pivot)
                    {
                        _current_pivot = _mth_abs(l_tDecouple.matrix_Real_To_Virtual[i][0]);
                        pivot_rows[0] = i;
                    }
                }

                for(uint8_t j = 0; j < _cfg_DECOUPLE_EQUATION_COLUMNS - 1; j++)
                {
                    A[i][j] = l_tDecouple.matrix_Real_To_Virtual[i][j];
                }
                A[i][_cfg_DECOUPLE_EQUATION_COLUMNS - 1] = _b[i];
            }

            {//FF
                printA(A);
            }

            //遍历所有，列主元高斯消元并得行阶梯
            for(uint8_t iter = 0; iter < _cfg_DECOUPLE_EQUATION_ROWS; iter++)
            {
                {//FF
                    cout<<"\niter = " <<(int)iter <<endl;
                    cout<<"other_rows == ";
                    for(uint8_t i = 0; i< _cfg_DECOUPLE_EQUATION_ROWS - iter; i++)
                        cout<<(int)other_rows[i] <<", ";
                    cout <<endl;
                }


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
                        //进入这里也意味着other_rows有效元素只有一个
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

                {//FF
                    cout<<"pivot_rows == " ;
                    for(uint8_t i = 0; i< iter + 1; i++)
                        cout<<(int)pivot_rows[i] <<", ";
                    cout <<endl;
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

                {//FF
                    printA(A);
                    cout <<endl;
                }
            }//end of 迭代

            //得出基底向量与特解
            float basis_vector[_cfg_DAC_GROUP_NUM] = {0,0,0,0};
            float special_solution[_cfg_DAC_GROUP_NUM] = {0,0,0,0};
            float sum_of_b_ = 0;
            float sum_of_s_ = 0;

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
            if(sum_of_b_ < 1e-3)
            {
                //在这里使用补充加和的方式，而不是重新计算sum
                //sum_of_b_ = 0;
                //sum_of_s_ = 0;

                uint8_t max_num_ = 0;
                float max_b_2_ = basis_vector[0] * basis_vector[0];
                for(uint8_t idx = 1; idx < _cfg_DAC_GROUP_NUM; idx++)
                {
                    float tmp = basis_vector[idx] * basis_vector[idx];
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
            float t_ = 0.0f - sum_of_s_/sum_of_b_;


            //求出最后的实电流
            for(uint8_t idx = 0; idx < _cfg_DAC_GROUP_NUM; idx++)
            {
                x[idx] = basis_vector[idx] * t_ + special_solution[idx];
            }


            {//FF
                cout <<"basis_vector: ";
                for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM; i++)
                {
                    cout<<basis_vector[i] <<", ";
                }
                cout <<endl;

                cout <<"special solution: ";
                for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM; i++)
                {
                    cout <<special_solution[i] <<", ";
                }
                cout <<endl;
                
                cout<< "t* = ";
                cout<<t_ <<endl;

                cout <<endl;
                cout <<"*************************************" <<endl;
                cout <<"i_real_group = ";
                for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM; i++)
                {
                    cout <<x[i] <<", ";
                }
                cout <<endl;
                cout <<"*************************************" <<endl;
                cout <<endl;
            }

            float i_real[_cfg_DAC_CHANNEL_NUM];
            for(uint8_t i = 0; i<_cfg_DAC_GROUP_NUM; i++)
            {
                for(uint8_t j = 0; j<_DAC_GROUP_X_NUM[i]; j++)
                {
                    i_real[_cfg_Group_of_DAC[i][j]-1] = x[i];
                }
            }

            {//FF
                cout << "i_DAC: ";
                for(uint8_t i = 0; i<_cfg_DAC_CHANNEL_NUM; i++)
                {
                    cout<<i_real[i] <<", ";
                }
                cout <<endl;
            }

        //end
        printA(A);
        }


    }


    return 0;
}