#ifndef __CONFIGURATION
#define __CONFIGURATION
#include "stm32f4xx.h"
#include "arm_math.h"



//调试模式开关（请保持开）
#define _cfg_DEBUG_MODE     0

//是否为测试功放模式（测试功放时为1，控制PID时为0）
#define _cfg_TEST_AMPLIFIER 0

//是否为测试解耦计算模式（仅测试解耦计算不真实输出则设为1，否则设为0）
#define _cfg_TEST_DECOUPLE  0

//是否为测试姿态测量模式（是则设为1）
#define _cfg_TEST_ORIENTATION   0




//结构定义
typedef enum{
    _ctl_STATUS_RUNNING = 0,
    _ctl_STATUS_STOP = 1
} SYSTEM_STATUS;
typedef enum{
    _ctl_MODE_ANGULAR_VELOCITY = 0,
    _ctl_MODE_ANGULAR_POSITION = 1,
    _ctl_MODE_TORQUE_STATOR_OUTPUT = 2,
    _ctl_MODE_TORQUE_ROTOR_OUTPUT = 3
} SYSTEM_MODE;
typedef struct{
    SYSTEM_STATUS STATUS;
    SYSTEM_MODE MODE;
} _ctl_SYSTEM_CONTROL;
//*******************************************************************************//
//**************************** 下层建筑部分(外设等) **************************************//
//*******************************************************************************//

//**************************** ADC 部分 **************************************//
//配置3个ADC中的开启情况
#define _cfg_USE_ADC_1 1
#define _cfg_USE_ADC_2 1
#define _cfg_USE_ADC_3 1
//配置采样率，也就是系统主时钟
#define _cst_SAMPLE_RATE_100KHZ     100000
#define _cst_SAMPLE_RATE_50khz      50000
#define _cst_SAMPLE_RATE_40KHZ      40000
#define _cst_SAMPLE_RATE_20KHZ      20000
#define _cst_SAMPLE_RATE_10KHZ      10000
#define _cst_SAMPLE_RATE_5KHZ       5000
// AD7606*8(*3) 最多用时：4+2+2+2 = 10us    DAC8563*2*4写入用时 20us 


//!!! VENTUSFF: 注意：此时，按照目前配置方式:
//  在主时钟开始后的第2个周期开始检测，检测到AD7606_2(/3)的BUSY引脚从高到低跳变时触发 FLAG
//  因此要保证在地2个周期能读到高电平，也就是要保证AD7606_2(/3)的采样时间超过主时钟 T，所以对过采样率有要求
//  这种情况下，若T = 50us，则能采用的过采样率为 100(16x) = 74us 之后
//  尝试改变这种情况，在第一个周期就立即检测高电平

/**************************************  时序部分  ******************************************
 * 主时钟：10khz，用于AD7606_1，DAC8563，次级控制系统
 * 次时钟：500hz，用于主控制系统，调试刷新等
 *      在SecondaryTask的Delay 中 塞入 MainTask： 
 *          一个ADNS9800的CS 低电平持续 500~600us，故预计一个Secondary Task大致 1.2ms
 * 
 * */
#define _cfg_MAIN_CLOCK            _cst_SAMPLE_RATE_10KHZ
#define _cfg_SECONDARY_CLOCK			 200
#define _cfg_PUT_MAIN_IN_SECONDARY	1


//从主时钟到次时钟的分频系数.
#define _cst_MAIN_DIVIDE_TO_SECONDARY       (_cfg_MAIN_CLOCK/_cfg_SECONDARY_CLOCK)
//从主时钟想要读取到AD7606_2,AD7606_3的最少需要的周期数(第n个周期时可以读取到读数)
#define _cst_MINIMUM_CLOCK_CYCLE_2_3        (_cfg_MAIN_CLOCK/_cfg_SECONDARY_CLOCK/2)


//***************************** DAC 部分 ********************************************//
//******************* 各线圈组 对应的DAC模块 端口及CS端引脚 ***************************//
//采用数据总线的形式；只有片选端不同，其他均公用
#define _cfg_DAC_CHANNEL_NUM    8

//******************* Grouping Method 线圈分组方式 ***************************//
//只有已经实现8路DAC，且目前需要只实现3自由度旋转时，需要考虑将8路组合为4路
//具体16组线圈如何组合为8路，只有硬件中才能体现->这里应该在设计的电路中体现出来

#define _cfg_COIL_NUM   16
#define _cfg_DAC_GROUP_NUM	4
extern const uint8_t _DAC_GROUP_X_NUM[_cfg_DAC_GROUP_NUM];
extern const uint8_t _cfg_Group_of_DAC[_cfg_DAC_GROUP_NUM][2];
extern const uint8_t _cfg_DAC_number_To_coil_number[_cfg_DAC_CHANNEL_NUM][2];
extern const int8_t _cfg_Directions[_cfg_DAC_CHANNEL_NUM][2];
extern const uint8_t _cfg_ENABLE_DAC[_cfg_DAC_CHANNEL_NUM];

//**************************** ADNS9800 部分 **************************************//
//CPI
#define _cfg_ADNS9800_CPI	6350

//*******************************************************************************//
//**************************** 控制系统部分 **************************************//
//*******************************************************************************//

//**************************** 主控制系统 **************************************//
/**
 * 输出电压精确度：(+-10V)/16bit*2倍 = 0.0006, 考虑误差，最终精度不超过0.001
 * 输入电流精确度：(+-5V)/16bit = 0.00015，考虑误差，最终精度不超过0.001
 * 基于以上考虑，用float计算足够. 而且上面的量也都是用16bit来存储的，而float 4字节，double 8字节，明显float足够
 * 故采用 float32_t
 * */

typedef float32_t _unit_MAIN_PID_DATA;
typedef _unit_MAIN_PID_DATA _unit_CURRENT_FLOAT_DATA;   //虚电流、实电流均保持与MAIN PID一致的运算精度
#define _cfg_LEVITATION_ON   0       //悬浮控制开关。实际上决定 SISO PID个数 = 虚电流维度 = 解算实电流的输入维度
#if _cfg_LEVITATION_ON
    #define _cfg_MAIN_PID_CHANNEL_NUM   6
#else
    #define _cfg_MAIN_PID_CHANNEL_NUM   3       //注意要与_cfg_COIL_GROUP_NUM 紧密配合
#endif

//增广矩阵的行数、列数
#define _cfg_DECOUPLE_EQUATION_ROWS _cfg_MAIN_PID_CHANNEL_NUM
#define _cfg_DECOUPLE_EQUATION_COLUMNS (_cfg_DAC_GROUP_NUM + 1)

//旋转控制：
#define _cfg_THRESHOLD     0.2f
#define _cfg_2_THRESHOLD     0.4f
#define _cfg_REACH_THRESHOLD    0.04f

//*************** 运算部分：三旋转角的解算 ****************//
/**运算规约
 * 1.长度型量，单位mm
 * 2.角度型量，单位rad
 * 3.运算时间估计：用<math.h>的算法，sin(3.141592)用了60微秒，sin(3.141592/2)用了40微秒。
 */
typedef int16_t _unit_MouseSensorINTData;
typedef float32_t _unit_MouseSensorFLOATData;
typedef float32_t _unit_RotateAngleData;
#define _cst_PI             3.141593f
#define _cst_2PI             6.283185f
#define _cst_ROTOR_RADIUS           30.0f
#define _cst_ROTOR_RADIUS_RECIPROCAL    0.03333333f
#define _cst_MOUSE_SENSOR_THETA     0.9424778f   //54degree*pi/180
#define _cst_ROTOR_RADIUS_X_MOUSE_SIN_THETA     24.27051f    //30.0*sin(54*pi/180)
#define _cst_ROTOR_RADIUS_X_MOUSE_SIN_THETA_RECIPROCAL  0.04120227f
#define _cst_COS_MOUSE_SENSOR_THETA             0.5877853f   //cos(54*pi/180)
#define _cst_MOUSE_SENSOR_SCALE_FACTOR_Y        1.030614f   //1/cos((50-(90-54))*pi/180)
#define _cst_RAD_TO_ANGLE           57.29578f                //180/pi
#define _mth_RAD_TO_ANGLE(x)        (x*_cst_RAD_TO_ANGLE)
#define _mth_abs(x)                 (x>0?x:0.0f - x)
#define _mth_sign(x)                (x>0?1:-1)
//*************** 运算部分：线圈在动子坐标系下位置的计算 ****************//
#define _cst_DELTA_ANGLE_EPSILONE_DEGREE    80              //500Hz下，假定一个周期最多运动度数  
#define _cst_COIL_COOR_RADIUS               0.037f           //球坐标下，37mm
extern const _unit_MAIN_PID_DATA _cst_COIL_POSITION[3*_cfg_COIL_NUM];
#define _cfg_DEBOUPLE_USE_FORMULA        0   //在解算4元一次方程组时，是否利用公式计算. (或利用计算方法)

//**************************** 次级控制系统 **************************************//
typedef _unit_MAIN_PID_DATA _unit_SUB_PID_DATA;     //次级控制系统与控制系统保持一致的运算精度


#define _cfg_USE_SUB_PID        1   //标识是否子控制系统是否使用PID（开环/PID闭环开关）
#define _cfg_USE_PRESET_PID     1   //是否使用预设的PID参数（或还是通过手动串口输入PID参数）

#define _cfg_USE_PRESET_PID_MAIN  0

//****** 开环 ******//
#define _cst_COIL_R         6.0f     //开环下，线圈电阻估测
#define _cst_CURRENT_MULT   2.0f     //开换下，放大倍数估测

//****** 闭环 ******//
//PID调节记录
/**
 * Kp = 2.0, Ki = 0.04      有多次震荡，无法达到幅值
 * Kp = 2.0, Ki = 0.02      有轻度震荡，无法达到幅值
 * Kp = 2.0, Ki = 0.01      震荡加剧，无法达到幅值
 * Kp = 3.0, Ki = 0.04      无法达到幅值震荡非常剧烈，
 * Kp = 2.0, Ki = 0.08      终于达到幅值；有多次震荡。另外发现电感似乎确实比想象中大。
 * Kp = 2.0, Ki = 0.1       超调10%
 * Kp = 2.5, Ki = 0.12      超调达到20%，震荡较大
 * Kp = 1.8, Ki = 0.1       超调8%
 * Kp = 2.5, Ki = 0.08      超调30%
 * Kp = 1.5, Ki = 0.07      达不到幅值，但接近
 * Kp = 1.5, Ki = 0.1       超调8%左右，震荡明显减轻，只有2次
 * Kp = 1.2, Ki = 0.1       超调5%，震荡只有1次
 * Kp = 1.1 -> 0.9 -> 0.6, Ki = 0.1    超调逐渐变大
 *^Kp = 1.2, Ki = 0.09      无超调，无震荡，响应时间200us
 * Kp = 1.2, Ki = 0.1 -> 0.09 -> 0.085 超调逐渐减小
 * Kp = 1.2, Ki = 0.085 -> 0.082 -> 0.08      无震荡，但刚开始达不到幅值有坑，越来越坑
 * Kp = 1.5, Ki = 0.08      刚开始达不到幅值，震荡还加剧了
 * 
 * 综上：Kp = 1.2时能达到幅值且基本没有震荡; Kp 增大或减小都会出现震荡
 * */

//Ki = 0.2时有多次震荡
#define _par_SUB_PID_KP     1.3f
#define _par_SUB_PID_KI     0.8f    //Ki = xe5 (因为Ts = 5e-5,故抵消)
#define _par_SUB_PID_KD     0.4f

//#define _par_SUB_PID_KI     0.06    //Ki = xe5 (因为Ts = 5e-5,故抵消)
//#define _par_SUB_PID_KP     10

#define _cfg_SUB_PID_CHANNEL_NUM _cfg_DAC_CHANNEL_NUM
#define _cfg_SUB_PID_COIL_CURRENT_MAGNITUDE     1.5f         //线圈电流幅度。
#define _cfg_SUB_PID_OUTPUT_VOLTATE_MAGNITUDE   7.5f         //放大前输出电压的赋值

#define _cst_SUB_PID_TS     5       //Ts = 5e-5
#define _cst_SUB_PID_OUTPUT_MAX     10.0f
#define _cst_SUB_PID_OUTPUT_ZERO    32767
#define _cst_DAC_1_BIT_TO_V        0.0003051758f    //20V/65536
#define _cst_DAC_1_V_TO_BIT        3276.8f          //65536/20V
#define _cst_ADC_1_BIT_TO_V        0.0001525879f    //10V/65536
#define _cst_ADC_1_V_TO_BIT        6553.6f          //65536/10V


//**************************** 时序检查部分 **************************************//
#define _tim_MAIN_CHECK_COUNT       _cfg_MAIN_CLOCK
#define _tim_SECONDARY_CHECK_COUNT  _cfg_SECONDARY_CLOCK
#define _tim_SECONDARY_TO_DEBUG     _cfg_SECONDARY_CLOCK/2

//**************************** 串口通信部分 **************************************//
#define _cfg_USART_BUFFER_MAX  1024
#define _cfg_USART_RECEIVE_END_FLAG '\r'
#define _cfg_USART_RECEIVE_END_FLAG_2 '\n'

//**************************** 归零装置部分 **************************************//
#define _cfg_USE_ZERO_DEVICE  1
#endif

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/
