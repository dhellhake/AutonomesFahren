/*
 * head.h
 *
 *  Created on: 04.07.2014
 *      Author: Hardik Shah
 */


#ifndef HEAD_H_
#define HEAD_H_

#include <system.h>
#include <alt_types.h>

#define PWM_ENABLE 				MOTOR_CONTROL_0_BASE
#define FRONT_RIGHT_DUTY_SET 	PWM_ENABLE+4
#define REAR_RIGHT_DUTY_SET 	FRONT_RIGHT_DUTY_SET+4
#define REAR_LEFT_DUTY_SET 		REAR_RIGHT_DUTY_SET+4
#define FRONT_LEFT_DUTY_SET 	REAR_LEFT_DUTY_SET+4

#define FRONT_RIGHT_ENC_SET 	FRONT_LEFT_DUTY_SET+4
#define REAR_RIGHT_ENC_SET 		FRONT_RIGHT_ENC_SET+4
#define REAR_LEFT_ENC_SET 		REAR_RIGHT_ENC_SET+4
#define FRONT_LEFT_ENC_SET 		REAR_LEFT_ENC_SET+4

#define FRONT_RIGHT_ENC_READ 	FRONT_LEFT_ENC_SET+4
#define REAR_RIGHT_ENC_READ 	FRONT_RIGHT_ENC_READ+4
#define REAR_LEFT_ENC_READ 		REAR_RIGHT_ENC_READ+4
#define FRONT_LEFT_ENC_READ 	REAR_LEFT_ENC_READ+4

#define PWM_RESOLUTION 10000

#define WHEEL_READY_MASK 0x8000
#define ALL_WHEEL_FWD_MASK 0x0055
#define ALL_WHEEL_BWD_MASK 0x00AA
#define TURN_RIGHT_MASK 0x005A
#define TURN_LEFT_MASK 0x00A5
#define ONLY_RIGHT_WHEELS_FWD_MASK 0x0005
#define ONLY_LEFT_WHEELS_FWD_MASK 0x0050
#define ONLY_RIGHT_WHEELS_BWD_MASK 0x000A
#define ONLY_LEFT_WHEELS_BWD_MASK 0x00A0
#define FRONT_RIGHT_WHEEL_FWD_MASK 0x0001
#define FRONT_RIGHT_WHEEL_BWD_MASK 0x0002
#define REAR_RIGHT_WHEEL_FWD_MASK 0x0004
#define REAR_RIGHT_WHEEL_BWD_MASK 0x0008
#define REAR_LEFT_WHEEL_FWD_MASK 0x0010
#define REAR_LEFT_WHEEL_BWD_MASK 0x0020
#define FRONT_LEFT_WHEEL_FWD_MASK 0x0040
#define FRONT_LEFT_WHEEL_BWD_MASK 0x0080
#define ENABLE_ESP_MASK 0x4000
#define PLAY_BACK_MASK 0x0100
#define ENABLE_ENC_MASK 0x00010000
#define PAUSE_ENC_MASK 0x00020000

#define ULTRA_SOUND_READY_ADDR HC_SR04_0_BASE
#define ULTRA_SOUND_TRIG_ADDR  HC_SR04_0_BASE
#define ULTRA_SOUND_TRIG_ALL_CMD  0xFF
#define ULTRA_SOUND_READY_ALL_MASK  0xFF

#define NUMBER_OF_ULTRA_SOUND_DEVICES 8
#define ULTRA_SOUND_RESOLUTION 6 // 0.3 cm resolution + 0.3 cm tolerance

#define ULTRA_SOUND_0_TRIG_CMD    0x1
#define ULTRA_SOUND_0_READY_MASK    0x1
#define ULTRA_SOUND_1_TRIG_CMD    0x2
#define ULTRA_SOUND_1_READY_MASK    0x2
#define ULTRA_SOUND_2_TRIG_CMD    0x4
#define ULTRA_SOUND_2_READY_MASK    0x4
#define ULTRA_SOUND_3_TRIG_CMD    0x8
#define ULTRA_SOUND_3_READY_MASK    0x8
#define ULTRA_SOUND_4_TRIG_CMD    0x10
#define ULTRA_SOUND_4_READY_MASK    0x10
#define ULTRA_SOUND_5_TRIG_CMD    0x20
#define ULTRA_SOUND_5_READY_MASK    0x20
#define ULTRA_SOUND_6_TRIG_CMD    0x40
#define ULTRA_SOUND_6_READY_MASK    0x40
#define ULTRA_SOUND_7_TRIG_CMD    0x80
#define ULTRA_SOUND_7_READY_MASK    0x80


#define ULTRA_SOUND_0_READING_ADDR  ULTRA_SOUND_TRIG_ADDR+4
#define ENCODER_RESOLUTION 5.184

#define MIN_PARKING_SLOT_WIDTH 260

#define MPU_SLAVE_ADDRESS 0x68
#define HMC5883L_SLAVE_ADDRESS 0x1E

#define I2C_SLAVE_ADDR (MP6050_0_BASE | 0x80000000)
#define I2C_REG_ADDR (I2C_SLAVE_ADDR + 1)
#define I2C_REG_WRITE_DATA (I2C_SLAVE_ADDR + 2)
#define I2C_DATALENGTH_ADDR (I2C_SLAVE_ADDR + 3)
#define I2C_CMD_ADDR (I2C_SLAVE_ADDR + 4)
#define I2C_DATA_AVAILABLE_ADDR (I2C_SLAVE_ADDR + 8)
#define I2C_READ_CMD_MASK 0x01
#define I2C_WRITE_CMD_MASK 0x00
#define I2C_START_CMD_MASK 0x02

alt_u8 c;
alt_u8 versionName[100];

unsigned int uPwmPeriod;

volatile unsigned int *pwm_enable;

volatile unsigned int *pFrontRightDutySet;
volatile unsigned int *pRearRightDutySet;
volatile unsigned int *pRearLeftDutySet;
volatile unsigned int *pFrontLeftDutySet;

volatile unsigned int *pFrontRightEncSet;
volatile unsigned int *pRearRightEncSet;
volatile unsigned int *pRearLeftEncSet;
volatile unsigned int *pFrontLeftEncSet;

volatile unsigned int *pFrontRightEncRead;
volatile unsigned int *pRearRightEncRead;
volatile unsigned int *pRearLeftEncRead;
volatile unsigned int *pFrontLeftEncRead;

volatile unsigned int *pHc_sr04;
volatile unsigned int  *pUltraSoundArray;

void init(void);
void delay (volatile unsigned int del);
unsigned int set_duty_cycle(volatile unsigned int *pTargetedWheel, unsigned int duty);
unsigned int MeasureDistance(unsigned int ch);

volatile unsigned char *pI2C;
volatile unsigned char *pI2CReg;
volatile unsigned char *pI2CRegWriteData;
volatile unsigned char *pI2CDataLen;
volatile unsigned char *pI2CCmd;
volatile unsigned char *pI2CDataAvailable;

void I2CWrite(unsigned char I2CAddr, unsigned char RegAddr, unsigned char data);
void I2CRead(unsigned char I2CAddr, unsigned char RegAddr, unsigned int len, char* cBuff);

#endif /* HEAD_H_ */
