/***************************************************************************************************
文件:   A9800Chip.h
说明:   A9800光电传感器芯片.A9800全称ADNS9800.
***************************************************************************************************/
#ifndef __A9800CHIP_H__  //防重包含.
#define __A9800CHIP_H__
/**************************************************************************************************/
/***************************************************************************************************
包含:   头文件.
***************************************************************************************************/
#include "stm32f4xx.h"

/***************************************************************************************************
定义:   寄存器.
***************************************************************************************************/
#define A9800_PRODUCTID_ADDR                            0x00
#define A9800_REVISIONID_ADDR                        0x01
#define A9800_MOTION_ADDR                            0x02
#define A9800_DELTAX_L_ADDR                            0x03
#define A9800_DELTAX_H_ADDR                            0x04
#define A9800_DELTAY_L_ADDR                            0x05
#define A9800_DELTAY_H_ADDR                            0x06
#define A9800_SQUAL_ADDR                                0x07
#define A9800_PIXEL_SUM_ADDR                            0x08
#define A9800_MAXIMUM_PIXEL_ADDR                        0x09
#define A9800_MINIMUM_PIXEL_ADDR                        0x0A
#define A9800_SHUTTER_LOWER_ADDR                        0x0B
#define A9800_SHUTTER_UPPER_ADDR                        0x0C
#define A9800_FRAME_PERIOD_LOWER_ADDR                0x0D
#define A9800_FRAME_PERIOD_UPPER_ADDR                0x0E
#define A9800_CONFIGURATION_I_ADDR                    0x0F
#define A9800_CONFIGURATION_II_ADDR                    0x10
#define A9800_FRAME_CAPTURE_ADDR                        0x12
#define A9800_SROM_ENABLE_ADDR                        0x13
#define A9800_RUN_DOWNSHIFT_ADDR                        0x14
#define A9800_REST1_RATE_ADDR                        0x15
#define A9800_REST1_DOWNSHIFT_ADDR                    0x16
#define A9800_REST2_RATE_ADDR                        0x17
#define A9800_REST2_DOWNSHIFT_ADDR                    0x18
#define A9800_REST3_RATE_ADDR                        0x19
#define A9800_FRAME_PERIOD_MAX_BOUND_LOWER_ADDR        0x1A
#define A9800_FRAME_PERIOD_MAX_BOUND_UPPER_ADDR        0x1B
#define A9800_FRAME_PERIOD_MIN_BOUND_LOWER_ADDR        0x1C
#define A9800_FRAME_PERIOD_MIN_BOUND_UPPER_ADDR        0x1D
#define A9800_SHUTTER_MAX_BOUND_LOWER_ADDR             0x1E
#define A9800_SHUTTER_MAX_BOUND_UPPER_ADDR             0x1F
#define A9800_LASER_CTRL0_ADDR                        0x20
#define A9800_LASER_CTRL1_ADDR                        0x21
#define A9800_LP_CFG0_ADDR                            0x22
#define A9800_LP_CFG1_ADDR                            0x23
#define A9800_OBSERVATION_ADDR                        0x24
#define A9800_DATA_OUT_LOWER_ADDR                    0x25
#define A9800_DATA_OUT_UPPER_ADDR                    0x26
#define A9800_SROM_ID_ADDR                            0x2A
#define A9800_LIFT_DETECTION_THR_ADDR                0x2E
#define A9800_CONFIGURATION_V_ADDR                    0x2F
#define A9800_CONFIGURATION_IV_ADDR                    0x39
#define A9800_POWER_UP_RESET_ADDR                    0x3A
#define A9800_SHUTDOWN_ADDR                            0x3B
#define A9800_INVERSE_PRODUCT_ID_ADDR                0x3F
#define A9800_MOTION_BURST_ADDR                        0x50
#define A9800_SROM_LOAD_BURST_ADDR                    0x62
#define A9800_PIXEL_BURST_ADDR                        0x64

/***************************************************************************************************
定义:   移动寄存器.
***************************************************************************************************/
#define A9800_MOTION_MOT            0x80
#define A9800_MOTION_FAULT          0x40
#define A9800_MOTION_LPVALID        0x20
#define A9800_MOTION_OVF            0x10
#define A9800_MOTION_PIXFIRST       0x01

/***************************************************************************************************
定义:   CPI编号.
***************************************************************************************************/
#define A9800_CPI_MIN               1
#define A9800_CPI_MAX               164

#define A9800_RES50CPI              1        
#define A9800_RES100CPI             2
#define A9800_RES150CPI             3

#define A9800_RES1000CPI            50
#define A9800_RES6350CPI            127
#define A9800_RES8200CPI            164

/***************************************************************************************************
定义:   抬起高度.
***************************************************************************************************/
#define A9800_LIFT_MIN              1
#define A9800_LIFT_MAX              31

// Configuration II Bits Register Individual Bit Field Settings
#define A9800_FREST1                0x80
#define A9800_FREST0                0x40
#define A9800_REST_EN            0x20
#define A9800_NAGC                0x10
#define A9800_FIXED_FR            0x08
#define A9800_RPT_MOD            0x04
#define A9800_SYS_TEST            0x01

// Laser CTRL0 Bits Register Individual Bit Field Settings
#define A9800_RANGE                0x80
#define A9800_FORCE_DISABLED        0x01

// Laser CTRL1 Bits Register Individual Bit Field Settings
#define A9800_RANGE_C            0x80

// Configuration IV Bits Register Individual Bit Field Settings
#define A9800_SROM_SIZE_1_5K        0x00
#define A9800_SROM_SIZE_3_0K        0x02

// Power Up Reset Register Settings
#define A9800_POWER_UP_RESET        0x5A

/**************************************************************************************************/
#endif //#ifndef __A9800CHIP_H__


/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/

