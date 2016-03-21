// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "I2Cdev.h"
#include "helper_3dmath.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "helper_3dmathc.h"
#ifdef __cplusplus
}
#endif
// supporting link:  http://forum.arduino.cc/index.php?&topic=143444.msg1079517#msg1079517
// also: http://forum.arduino.cc/index.php?&topic=141571.msg1062899#msg1062899s

//#ifndef __arm__
//#include <avr/pgmspace.h>
//#else
#define PROGMEM /* empty */
#define pgm_read_byte(x) (*(x))
#define pgm_read_word(x) (*(x))
#define pgm_read_float(x) (*(x))
#define PSTR(STR) STR
//#endif

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_SELF_TEST_XA_1_BIT     0x07
#define MPU6050_SELF_TEST_XA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_XA_2_BIT     0x05
#define MPU6050_SELF_TEST_XA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_YA_1_BIT     0x07
#define MPU6050_SELF_TEST_YA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_YA_2_BIT     0x03
#define MPU6050_SELF_TEST_YA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_ZA_1_BIT     0x07
#define MPU6050_SELF_TEST_ZA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_ZA_2_BIT     0x01
#define MPU6050_SELF_TEST_ZA_2_LENGTH  0x02

#define MPU6050_SELF_TEST_XG_1_BIT     0x04
#define MPU6050_SELF_TEST_XG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_YG_1_BIT     0x04
#define MPU6050_SELF_TEST_YG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_ZG_1_BIT     0x04
#define MPU6050_SELF_TEST_ZG_1_LENGTH  0x05

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   4

#ifdef __cplusplus
// note: DMP code memory blocks defined at end of header file

class MPU6050 {
public:
	MPU6050();
	MPU6050(unsigned char address);
	void initialize();
	bool testConnection();

	// AUX_VDDIO register
	unsigned char getAuxVDDIOLevel();
	void setAuxVDDIOLevel(unsigned char level);

	// SMPLRT_DIV register
	unsigned char getRate();
	void setRate(unsigned char rate);

	// CONFIG register
	unsigned char getExternalFrameSync();
	void setExternalFrameSync(unsigned char sync);
	unsigned char getDLPFMode();
	void setDLPFMode(unsigned char bandwidth);

	// GYRO_CONFIG register
	unsigned char getFullScaleGyroRange();
	void setFullScaleGyroRange(unsigned char range);

	// SELF_TEST registers
	unsigned char getAccelXSelfTestFactoryTrim();
	unsigned char getAccelYSelfTestFactoryTrim();
	unsigned char getAccelZSelfTestFactoryTrim();

	unsigned char getGyroXSelfTestFactoryTrim();
	unsigned char getGyroYSelfTestFactoryTrim();
	unsigned char getGyroZSelfTestFactoryTrim();

	// ACCEL_CONFIG register
	bool getAccelXSelfTest();
	void setAccelXSelfTest(bool enabled);
	bool getAccelYSelfTest();
	void setAccelYSelfTest(bool enabled);
	bool getAccelZSelfTest();
	void setAccelZSelfTest(bool enabled);
	unsigned char getFullScaleAccelRange();
	void setFullScaleAccelRange(unsigned char range);
	unsigned char getDHPFMode();
	void setDHPFMode(unsigned char mode);

	// FF_THR register
	unsigned char getFreefallDetectionThreshold();
	void setFreefallDetectionThreshold(unsigned char threshold);

	// FF_DUR register
	unsigned char getFreefallDetectionDuration();
	void setFreefallDetectionDuration(unsigned char duration);

	// MOT_THR register
	unsigned char getMotionDetectionThreshold();
	void setMotionDetectionThreshold(unsigned char threshold);

	// MOT_DUR register
	unsigned char getMotionDetectionDuration();
	void setMotionDetectionDuration(unsigned char duration);

	// ZRMOT_THR register
	unsigned char getZeroMotionDetectionThreshold();
	void setZeroMotionDetectionThreshold(unsigned char threshold);

	// ZRMOT_DUR register
	unsigned char getZeroMotionDetectionDuration();
	void setZeroMotionDetectionDuration(unsigned char duration);

	// FIFO_EN register
	bool getTempFIFOEnabled();
	void setTempFIFOEnabled(bool enabled);
	bool getXGyroFIFOEnabled();
	void setXGyroFIFOEnabled(bool enabled);
	bool getYGyroFIFOEnabled();
	void setYGyroFIFOEnabled(bool enabled);
	bool getZGyroFIFOEnabled();
	void setZGyroFIFOEnabled(bool enabled);
	bool getAccelFIFOEnabled();
	void setAccelFIFOEnabled(bool enabled);
	bool getSlave2FIFOEnabled();
	void setSlave2FIFOEnabled(bool enabled);
	bool getSlave1FIFOEnabled();
	void setSlave1FIFOEnabled(bool enabled);
	bool getSlave0FIFOEnabled();
	void setSlave0FIFOEnabled(bool enabled);

	// I2C_MST_CTRL register
	bool getMultiMasterEnabled();
	void setMultiMasterEnabled(bool enabled);
	bool getWaitForExternalSensorEnabled();
	void setWaitForExternalSensorEnabled(bool enabled);
	bool getSlave3FIFOEnabled();
	void setSlave3FIFOEnabled(bool enabled);
	bool getSlaveReadWriteTransitionEnabled();
	void setSlaveReadWriteTransitionEnabled(bool enabled);
	unsigned char getMasterClockSpeed();
	void setMasterClockSpeed(unsigned char speed);

	// I2C_SLV* registers (Slave 0-3)
	unsigned char getSlaveAddress(unsigned char num);
	void setSlaveAddress(unsigned char num, unsigned char address);
	unsigned char getSlaveRegister(unsigned char num);
	void setSlaveRegister(unsigned char num, unsigned char reg);
	bool getSlaveEnabled(unsigned char num);
	void setSlaveEnabled(unsigned char num, bool enabled);
	bool getSlaveWordByteSwap(unsigned char num);
	void setSlaveWordByteSwap(unsigned char num, bool enabled);
	bool getSlaveWriteMode(unsigned char num);
	void setSlaveWriteMode(unsigned char num, bool mode);
	bool getSlaveWordGroupOffset(unsigned char num);
	void setSlaveWordGroupOffset(unsigned char num, bool enabled);
	unsigned char getSlaveDataLength(unsigned char num);
	void setSlaveDataLength(unsigned char num, unsigned char length);

	// I2C_SLV* registers (Slave 4)
	unsigned char getSlave4Address();
	void setSlave4Address(unsigned char address);
	unsigned char getSlave4Register();
	void setSlave4Register(unsigned char reg);
	void setSlave4OutputByte(unsigned char data);
	bool getSlave4Enabled();
	void setSlave4Enabled(bool enabled);
	bool getSlave4InterruptEnabled();
	void setSlave4InterruptEnabled(bool enabled);
	bool getSlave4WriteMode();
	void setSlave4WriteMode(bool mode);
	unsigned char getSlave4MasterDelay();
	void setSlave4MasterDelay(unsigned char delay);
	unsigned char getSlate4InputByte();

	// I2C_MST_STATUS register
	bool getPassthroughStatus();
	bool getSlave4IsDone();
	bool getLostArbitration();
	bool getSlave4Nack();
	bool getSlave3Nack();
	bool getSlave2Nack();
	bool getSlave1Nack();
	bool getSlave0Nack();

	// INT_PIN_CFG register
	bool getInterruptMode();
	void setInterruptMode(bool mode);
	bool getInterruptDrive();
	void setInterruptDrive(bool drive);
	bool getInterruptLatch();
	void setInterruptLatch(bool latch);
	bool getInterruptLatchClear();
	void setInterruptLatchClear(bool clear);
	bool getFSyncInterruptLevel();
	void setFSyncInterruptLevel(bool level);
	bool getFSyncInterruptEnabled();
	void setFSyncInterruptEnabled(bool enabled);
	bool getI2CBypassEnabled();
	void setI2CBypassEnabled(bool enabled);
	bool getClockOutputEnabled();
	void setClockOutputEnabled(bool enabled);

	// INT_ENABLE register
	unsigned char getIntEnabled();
	void setIntEnabled(unsigned char enabled);
	bool getIntFreefallEnabled();
	void setIntFreefallEnabled(bool enabled);
	bool getIntMotionEnabled();
	void setIntMotionEnabled(bool enabled);
	bool getIntZeroMotionEnabled();
	void setIntZeroMotionEnabled(bool enabled);
	bool getIntFIFOBufferOverflowEnabled();
	void setIntFIFOBufferOverflowEnabled(bool enabled);
	bool getIntI2CMasterEnabled();
	void setIntI2CMasterEnabled(bool enabled);
	bool getIntDataReadyEnabled();
	void setIntDataReadyEnabled(bool enabled);

	// INT_STATUS register
	unsigned char getIntStatus();
	bool getIntFreefallStatus();
	bool getIntMotionStatus();
	bool getIntZeroMotionStatus();
	bool getIntFIFOBufferOverflowStatus();
	bool getIntI2CMasterStatus();
	bool getIntDataReadyStatus();

	// ACCEL_*OUT_* registers
	void getMotion9(short* ax, short* ay, short* az, short* gx, short* gy,
			short* gz, short* mx, short* my, short* mz);
	void getMotion6(short* ax, short* ay, short* az, short* gx, short* gy,
			short* gz);
	void getAcceleration(short* x, short* y, short* z);
	short getAccelerationX();
	short getAccelerationY();
	short getAccelerationZ();

	// TEMP_OUT_* registers
	short getTemperature();

	// GYRO_*OUT_* registers
	void getRotation(short* x, short* y, short* z);
	short getRotationX();
	short getRotationY();
	short getRotationZ();

	// EXT_SENS_DATA_* registers
	unsigned char getExternalSensorByte(int position);
	unsigned short getExternalSensorWord(int position);
	unsigned long getExternalSensorDWord(int position);

	// MOT_DETECT_STATUS register
	unsigned char getMotionStatus();
	bool getXNegMotionDetected();
	bool getXPosMotionDetected();
	bool getYNegMotionDetected();
	bool getYPosMotionDetected();
	bool getZNegMotionDetected();
	bool getZPosMotionDetected();
	bool getZeroMotionDetected();

	// I2C_SLV*_DO register
	void setSlaveOutputByte(unsigned char num, unsigned char data);

	// I2C_MST_DELAY_CTRL register
	bool getExternalShadowDelayEnabled();
	void setExternalShadowDelayEnabled(bool enabled);
	bool getSlaveDelayEnabled(unsigned char num);
	void setSlaveDelayEnabled(unsigned char num, bool enabled);

	// SIGNAL_PATH_RESET register
	void resetGyroscopePath();
	void resetAccelerometerPath();
	void resetTemperaturePath();

	// MOT_DETECT_CTRL register
	unsigned char getAccelerometerPowerOnDelay();
	void setAccelerometerPowerOnDelay(unsigned char delay);
	unsigned char getFreefallDetectionCounterDecrement();
	void setFreefallDetectionCounterDecrement(unsigned char decrement);
	unsigned char getMotionDetectionCounterDecrement();
	void setMotionDetectionCounterDecrement(unsigned char decrement);

	// USER_CTRL register
	bool getFIFOEnabled();
	void setFIFOEnabled(bool enabled);
	bool getI2CMasterModeEnabled();
	void setI2CMasterModeEnabled(bool enabled);
	void switchSPIEnabled(bool enabled);
	void resetFIFO();
	void resetI2CMaster();
	void resetSensors();

	// PWR_MGMT_1 register
	void reset();
	bool getSleepEnabled();
	void setSleepEnabled(bool enabled);
	bool getWakeCycleEnabled();
	void setWakeCycleEnabled(bool enabled);
	bool getTempSensorEnabled();
	void setTempSensorEnabled(bool enabled);
	unsigned char getClockSource();
	void setClockSource(unsigned char source);

	// PWR_MGMT_2 register
	unsigned char getWakeFrequency();
	void setWakeFrequency(unsigned char frequency);
	bool getStandbyXAccelEnabled();
	void setStandbyXAccelEnabled(bool enabled);
	bool getStandbyYAccelEnabled();
	void setStandbyYAccelEnabled(bool enabled);
	bool getStandbyZAccelEnabled();
	void setStandbyZAccelEnabled(bool enabled);
	bool getStandbyXGyroEnabled();
	void setStandbyXGyroEnabled(bool enabled);
	bool getStandbyYGyroEnabled();
	void setStandbyYGyroEnabled(bool enabled);
	bool getStandbyZGyroEnabled();
	void setStandbyZGyroEnabled(bool enabled);

	// FIFO_COUNT_* registers
	unsigned short getFIFOCount();

	// FIFO_R_W register
	unsigned char getFIFOByte();
	void setFIFOByte(unsigned char data);
	void getFIFOBytes(unsigned char *data, unsigned char length);

	// WHO_AM_I register
	unsigned char getDeviceID();
	void setDeviceID(unsigned char id);

	// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

	// XG_OFFS_TC register
	unsigned char getOTPBankValid();
	void setOTPBankValid(bool enabled);
	char getXGyroOffsetTC();
	void setXGyroOffsetTC(char offset);

	// YG_OFFS_TC register
	char getYGyroOffsetTC();
	void setYGyroOffsetTC(char offset);

	// ZG_OFFS_TC register
	char getZGyroOffsetTC();
	void setZGyroOffsetTC(char offset);

	// X_FINE_GAIN register
	char getXFineGain();
	void setXFineGain(char gain);

	// Y_FINE_GAIN register
	char getYFineGain();
	void setYFineGain(char gain);

	// Z_FINE_GAIN register
	char getZFineGain();
	void setZFineGain(char gain);

	// XA_OFFS_* registers
	short getXAccelOffset();
	void setXAccelOffset(short offset);

	// YA_OFFS_* register
	short getYAccelOffset();
	void setYAccelOffset(short offset);

	// ZA_OFFS_* register
	short getZAccelOffset();
	void setZAccelOffset(short offset);

	// XG_OFFS_USR* registers
	short getXGyroOffset();
	void setXGyroOffset(short offset);

	// YG_OFFS_USR* register
	short getYGyroOffset();
	void setYGyroOffset(short offset);

	// ZG_OFFS_USR* register
	short getZGyroOffset();
	void setZGyroOffset(short offset);

	// INT_ENABLE register (DMP functions)
	bool getIntPLLReadyEnabled();
	void setIntPLLReadyEnabled(bool enabled);
	bool getIntDMPEnabled();
	void setIntDMPEnabled(bool enabled);

	// DMP_INT_STATUS
	bool getDMPInt5Status();
	bool getDMPInt4Status();
	bool getDMPInt3Status();
	bool getDMPInt2Status();
	bool getDMPInt1Status();
	bool getDMPInt0Status();

	// INT_STATUS register (DMP functions)
	bool getIntPLLReadyStatus();
	bool getIntDMPStatus();

	// USER_CTRL register (DMP functions)
	bool getDMPEnabled();
	void setDMPEnabled(bool enabled);
	void resetDMP();

	// BANK_SEL register
	void setMemoryBank(unsigned char bank, bool prefetchEnabled = false,
			bool userBank = false);

	// MEM_START_ADDR register
	void setMemoryStartAddress(unsigned char address);

	// MEM_R_W register
	unsigned char readMemoryByte();
	void writeMemoryByte(unsigned char data);
	void readMemoryBlock(unsigned char *data, unsigned short dataSize,
			unsigned char bank = 0, unsigned char address = 0);
	bool writeMemoryBlock(const unsigned char *data, unsigned short dataSize,
			unsigned char bank = 0, unsigned char address = 0, bool verify =
					true, bool useProgMem = false);
	bool writeProgMemoryBlock(const unsigned char *data,
			unsigned short dataSize, unsigned char bank = 0,
			unsigned char address = 0, bool verify = true);

	bool writeDMPConfigurationSet(const unsigned char *data,
			unsigned short dataSize, bool useProgMem = false);
	bool writeProgDMPConfigurationSet(const unsigned char *data,
			unsigned short dataSize);

	// DMP_CFG_1 register
	unsigned char getDMPConfig1();
	void setDMPConfig1(unsigned char config);

	// DMP_CFG_2 register
	unsigned char getDMPConfig2();
	void setDMPConfig2(unsigned char config);

	// special methods for MotionApps 2.0 implementation
	//#ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS20
	unsigned char *dmpPacketBuffer;
	unsigned short dmpPacketSize;

	unsigned char dmpInitialize();
	bool dmpPacketAvailable();

	unsigned char dmpSetFIFORate(unsigned char fifoRate);
	unsigned char dmpGetFIFORate();
	unsigned char dmpGetSampleStepSizeMS();
	unsigned char dmpGetSampleFrequency();
	long dmpDecodeTemperature(char tempReg);

	// Register callbacks after a packet of FIFO data is processed
	//unsigned char dmpRegisterFIFORateProcess(inv_obj_func func, short priority);
	//unsigned char dmpUnregisterFIFORateProcess(inv_obj_func func);
	unsigned char dmpRunFIFORateProcesses();

	// Setup FIFO for various output
	unsigned char dmpSendQuaternion(unsigned short accuracy);
	unsigned char dmpSendGyro(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendAccel(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendLinearAccel(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendLinearAccelInWorld(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendControlData(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendSensorData(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendExternalSensorData(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendGravity(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendPacketNumber(unsigned short accuracy);
	unsigned char dmpSendQuantizedAccel(unsigned short elements,
			unsigned short accuracy);
	unsigned char dmpSendEIS(unsigned short elements, unsigned short accuracy);

	// Get Fixed Point data from FIFO
	unsigned char dmpGetAccel(long *data, const unsigned char* packet = 0);
	unsigned char dmpGetAccel(short *data, const unsigned char* packet = 0);
	unsigned char dmpGetAccel(VectorInt16 *v, const unsigned char* packet = 0);
	unsigned char dmpGetQuaternion(long *data, const unsigned char* packet = 0);
	unsigned char dmpGetQuaternion(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetQuaternion(Quaternion *q, const unsigned char* packet =
			0);
	unsigned char dmpGet6AxisQuaternion(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGet6AxisQuaternion(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGet6AxisQuaternion(Quaternion *q,
			const unsigned char* packet = 0);
	unsigned char dmpGetRelativeQuaternion(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetRelativeQuaternion(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetRelativeQuaternion(Quaternion *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetGyro(long *data, const unsigned char* packet = 0);
	unsigned char dmpGetGyro(short *data, const unsigned char* packet = 0);
	unsigned char dmpGetGyro(VectorInt16 *v, const unsigned char* packet = 0);
	unsigned char dmpSetLinearAccelFilterCoefficient(float coef);
	unsigned char dmpGetLinearAccel(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetLinearAccel(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetLinearAccel(VectorInt16 *v,
			const unsigned char* packet = 0);
	unsigned char dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw,
			VectorFloat *gravity);
	unsigned char dmpGetLinearAccelInWorld(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetLinearAccelInWorld(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetLinearAccelInWorld(VectorInt16 *v,
			const unsigned char* packet = 0);
	unsigned char dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal,
			Quaternion *q);
	unsigned char dmpGetGyroAndAccelSensor(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetGyroAndAccelSensor(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a,
			const unsigned char* packet = 0);
	unsigned char dmpGetGyroSensor(long *data, const unsigned char* packet = 0);
	unsigned char dmpGetGyroSensor(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetGyroSensor(VectorInt16 *v, const unsigned char* packet =
			0);
	unsigned char dmpGetControlData(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetTemperature(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetGravity(long *data, const unsigned char* packet = 0);
	unsigned char dmpGetGravity(short *data, const unsigned char* packet = 0);
	unsigned char dmpGetGravity(VectorInt16 *v,
			const unsigned char* packet = 0);
	unsigned char dmpGetGravity(VectorFloat *v, Quaternion *q);
	unsigned char dmpGetUnquantizedAccel(long *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetUnquantizedAccel(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetUnquantizedAccel(VectorInt16 *v,
			const unsigned char* packet = 0);
	unsigned char dmpGetQuantizedAccel(long *data, const unsigned char* packet =
			0);
	unsigned char dmpGetQuantizedAccel(short *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetQuantizedAccel(VectorInt16 *v,
			const unsigned char* packet = 0);
	unsigned char dmpGetExternalSensorData(long *data, unsigned short size,
			const unsigned char* packet = 0);
	unsigned char dmpGetEIS(long *data, const unsigned char* packet = 0);

	unsigned char dmpGetEuler(float *data, Quaternion *q);
	unsigned char dmpGetYawPitchRoll(float *data, Quaternion *q,
			VectorFloat *gravity);

	// Get Floating Point data from FIFO
	unsigned char dmpGetAccelFloat(float *data,
			const unsigned char* packet = 0);
	unsigned char dmpGetQuaternionFloat(float *data,
			const unsigned char* packet = 0);

	unsigned char dmpProcessFIFOPacket(const unsigned char *dmpData);
	unsigned char dmpReadAndProcessFIFOPacket(unsigned char numPackets,
			unsigned char *processed = 0);

	unsigned char dmpSetFIFOProcessedCallback(void (*func)(void));

	unsigned char dmpInitFIFOParam();
	unsigned char dmpCloseFIFO();
	unsigned char dmpSetGyroDataSource(unsigned char source);
	unsigned char dmpDecodeQuantizedAccel();
	unsigned long dmpGetGyroSumOfSquare();
	unsigned long dmpGetAccelSumOfSquare();
	void dmpOverrideQuaternion(long *q);
	unsigned short dmpGetFIFOPacketSize();
	//#endif

	// special methods for MotionApps 4.1 implementation
#ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS41
	unsigned char *dmpPacketBuffer;
	unsigned short dmpPacketSize;

	unsigned char dmpInitialize();
	bool dmpPacketAvailable();

	unsigned char dmpSetFIFORate(unsigned char fifoRate);
	unsigned char dmpGetFIFORate();
	unsigned char dmpGetSampleStepSizeMS();
	unsigned char dmpGetSampleFrequency();
	int32_t dmpDecodeTemperature(char tempReg);

	// Register callbacks after a packet of FIFO data is processed
	//unsigned char dmpRegisterFIFORateProcess(inv_obj_func func, short priority);
	//unsigned char dmpUnregisterFIFORateProcess(inv_obj_func func);
	unsigned char dmpRunFIFORateProcesses();

	// Setup FIFO for various output
	unsigned char dmpSendQuaternion(unsigned short accuracy);
	unsigned char dmpSendGyro(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendAccel(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendLinearAccel(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendLinearAccelInWorld(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendControlData(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendSensorData(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendExternalSensorData(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendGravity(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendPacketNumber(unsigned short accuracy);
	unsigned char dmpSendQuantizedAccel(unsigned short elements, unsigned short accuracy);
	unsigned char dmpSendEIS(unsigned short elements, unsigned short accuracy);

	// Get Fixed Point data from FIFO
	unsigned char dmpGetAccel(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetAccel(short *data, const unsigned char* packet=0);
	unsigned char dmpGetAccel(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetQuaternion(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetQuaternion(short *data, const unsigned char* packet=0);
	unsigned char dmpGetQuaternion(Quaternion *q, const unsigned char* packet=0);
	unsigned char dmpGet6AxisQuaternion(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGet6AxisQuaternion(short *data, const unsigned char* packet=0);
	unsigned char dmpGet6AxisQuaternion(Quaternion *q, const unsigned char* packet=0);
	unsigned char dmpGetRelativeQuaternion(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetRelativeQuaternion(short *data, const unsigned char* packet=0);
	unsigned char dmpGetRelativeQuaternion(Quaternion *data, const unsigned char* packet=0);
	unsigned char dmpGetGyro(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetGyro(short *data, const unsigned char* packet=0);
	unsigned char dmpGetGyro(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetMag(short *data, const unsigned char* packet=0);
	unsigned char dmpSetLinearAccelFilterCoefficient(float coef);
	unsigned char dmpGetLinearAccel(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetLinearAccel(short *data, const unsigned char* packet=0);
	unsigned char dmpGetLinearAccel(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
	unsigned char dmpGetLinearAccelInWorld(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetLinearAccelInWorld(short *data, const unsigned char* packet=0);
	unsigned char dmpGetLinearAccelInWorld(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
	unsigned char dmpGetGyroAndAccelSensor(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetGyroAndAccelSensor(short *data, const unsigned char* packet=0);
	unsigned char dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const unsigned char* packet=0);
	unsigned char dmpGetGyroSensor(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetGyroSensor(short *data, const unsigned char* packet=0);
	unsigned char dmpGetGyroSensor(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetControlData(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetTemperature(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetGravity(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetGravity(short *data, const unsigned char* packet=0);
	unsigned char dmpGetGravity(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetGravity(VectorFloat *v, Quaternion *q);
	unsigned char dmpGetUnquantizedAccel(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetUnquantizedAccel(short *data, const unsigned char* packet=0);
	unsigned char dmpGetUnquantizedAccel(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetQuantizedAccel(int32_t *data, const unsigned char* packet=0);
	unsigned char dmpGetQuantizedAccel(short *data, const unsigned char* packet=0);
	unsigned char dmpGetQuantizedAccel(VectorInt16 *v, const unsigned char* packet=0);
	unsigned char dmpGetExternalSensorData(int32_t *data, unsigned short size, const unsigned char* packet=0);
	unsigned char dmpGetEIS(int32_t *data, const unsigned char* packet=0);

	unsigned char dmpGetEuler(float *data, Quaternion *q);
	unsigned char dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

	// Get Floating Point data from FIFO
	unsigned char dmpGetAccelFloat(float *data, const unsigned char* packet=0);
	unsigned char dmpGetQuaternionFloat(float *data, const unsigned char* packet=0);

	unsigned char dmpProcessFIFOPacket(const unsigned char *dmpData);
	unsigned char dmpReadAndProcessFIFOPacket(unsigned char numPackets, unsigned char *processed=NULL);

	unsigned char dmpSetFIFOProcessedCallback(void (*func) (void));

	unsigned char dmpInitFIFOParam();
	unsigned char dmpCloseFIFO();
	unsigned char dmpSetGyroDataSource(unsigned char source);
	unsigned char dmpDecodeQuantizedAccel();
	unsigned long dmpGetGyroSumOfSquare();
	unsigned long dmpGetAccelSumOfSquare();
	void dmpOverrideQuaternion(long *q);
	unsigned short dmpGetFIFOPacketSize();
#endif

private:
	unsigned char devAddr;
	unsigned char buffer[14];
};

#endif

#ifdef __cplusplus
extern "C" {
#endif
		void* getMPU();
		unsigned char mpuDmpInitialize(void *mpu);
		void initMPU(void *mpu);
		short mpuTestConnection(void *mpu);
		unsigned char mpuDmpReadAndProcessFIFOPacket(void *mpu, unsigned char numPackets, unsigned char *processed);
		void mpuSetDMPEnabled(void *mpu, short enabled);
		unsigned short mpuGetFIFOCount(void *mpu);
		unsigned short mpuDmpGetFIFOPacketSize(void *mpu);
		void mpuGetFIFOBytes(void *mpu, unsigned char *data, unsigned char length);

		unsigned char mpuGetIntStatus(void *mpu);
		void mpuResetFIFO(void *mpu);

		void mpuSetXGyroOffset(void *mpu, short offset);
		void mpuSetYGyroOffset(void *mpu, short offset);
		void mpuSetZGyroOffset(void *mpu, short offset);

		void mpuSetXAccelOffset(void *mpu, short offset);
		void mpuSetYAccelOffset(void *mpu, short offset);
		void mpuSetZAccelOffset(void *mpu, short offset);

		CVectorInt16 mpuGetGyro(void *mpu, const unsigned char* packet);
		unsigned char mpuDmpGetQuaternion(void *mpu, CQuaternion *q, const unsigned char* packet);
		unsigned char mpuDmpGetGravity(void *mpu, CVectorFloat *v, CQuaternion *q);
		unsigned char mpuDmpGetYawPitchRoll(void *mpu, float *data, CQuaternion *q, CVectorFloat *gravity);
		unsigned char mpuDmpGetAccel(void *mpu, CVectorInt16 *v, const unsigned char* packet);
		unsigned char mpuDmpGetEuler(void *mpu, float *data, CQuaternion *q);

		void mpuGetMotion6(void *mpu, int* ax, int* ay, int* az, int* gx, int* gy,
				int* gz);

		void calibrateMPU(void *mpu);
		void meanSensors(void *mpu, int *meanAx, int *meanAy, int *meanAz, int *meanGx, int *meanGy, int *meanGz);
		void callibration(void *mpu, int *meanAx, int *meanAy, int *meanAz, int *meanGx, int *meanGy, int *meanGz, short *ax_offset, short *ay_offset, short *az_offset, short *gx_offset, short *gy_offset, short *gz_offset);
		void mpuDelay (volatile unsigned int del);

#ifdef __cplusplus
}
#endif
/*
extern "C" void call_MPU_initialize(MPU6050* p) // wrapper function
		{
	p->initialize();
}
*/
#endif /* _MPU6050_H_ */
