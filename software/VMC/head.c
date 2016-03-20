/*@file
 *@brief head.c
*/

#include "head.h"

void init(void)
{
	//c = 33;
	//versionName[100] = "Nano Board Version 1.0\n";
	pwm_enable = (volatile unsigned int*)(0x80000000 | PWM_ENABLE);

	pFrontRightDutySet = (volatile unsigned int*)(0x80000000 | FRONT_RIGHT_DUTY_SET);
	pRearRightDutySet = (volatile unsigned int*)(0x80000000 | REAR_RIGHT_DUTY_SET);
	pRearLeftDutySet = (volatile unsigned int*)(0x80000000 | REAR_LEFT_DUTY_SET);
	pFrontLeftDutySet = (volatile unsigned int*)(0x80000000 | FRONT_LEFT_DUTY_SET);

	pFrontRightEncSet = (volatile unsigned int*)(0x80000000 |FRONT_RIGHT_ENC_SET);
	pRearRightEncSet = (volatile unsigned int*)(0x80000000 | REAR_RIGHT_ENC_SET);
	pRearLeftEncSet = (volatile unsigned int*)(0x80000000 | REAR_LEFT_ENC_SET);
	pFrontLeftEncSet = (volatile unsigned int*)(0x80000000 | FRONT_LEFT_ENC_SET);

	pFrontRightEncRead = (volatile unsigned int*)(0x80000000 |FRONT_RIGHT_ENC_READ);
	pRearRightEncRead = (volatile unsigned int*)(0x80000000 |REAR_RIGHT_ENC_READ);
	pRearLeftEncRead = (volatile unsigned int*)(0x80000000 |REAR_LEFT_ENC_READ);
	pFrontLeftEncRead = (volatile unsigned int*)(0x80000000 | FRONT_LEFT_ENC_READ);

	uPwmPeriod = NIOS2_CPU_FREQ/PWM_RESOLUTION;

	pHc_sr04 = (volatile unsigned int*)(HC_SR04_0_BASE | 0x80000000);
	pUltraSoundArray = (volatile unsigned int*)(ULTRA_SOUND_0_READING_ADDR | 0x80000000);

    pI2C = (volatile unsigned char*)(I2C_SLAVE_ADDR);
    pI2CReg = (volatile unsigned char*)(I2C_REG_ADDR);
    pI2CRegWriteData = (volatile unsigned char*)(I2C_REG_WRITE_DATA);
    pI2CDataLen = (volatile unsigned char*)(I2C_DATALENGTH_ADDR);
    pI2CCmd = (volatile unsigned char*)(I2C_CMD_ADDR);
    pI2CDataAvailable = (volatile unsigned char*)(I2C_DATA_AVAILABLE_ADDR);
}


void delay (volatile unsigned int del)
{
	while (del != 0)
	{
		del --;
	}
}

unsigned int set_duty_cycle(volatile unsigned int *pTargetedWheel, unsigned int duty)
{
	if (duty > 100 || duty < 0) return(0);

	*pTargetedWheel = (uPwmPeriod/100) * duty; // duty is a positive number. Divide by 100 to consider percentage
}

unsigned int MeasureDistance(unsigned int ch)
{
	float x = (float)pUltraSoundArray[ch] * 330000/ALT_CPU_FREQ;
	return( (unsigned int)(x/2) );
}

void I2CWrite(unsigned char I2CAddr, unsigned char RegAddr, unsigned char data)
{
	*pI2C = I2CAddr; // FIrst write which I2C module is being addressed. MPU or HMC
	*pI2CReg = RegAddr; // write the address of register to which you want to write
	*pI2CRegWriteData = data; // write the data
	*pI2CDataLen = 1; // currently only 1 is supported for write access
	*pI2CCmd = (I2C_WRITE_CMD_MASK | I2C_START_CMD_MASK); // give write+start command
}

void I2CRead(unsigned char I2CAddr, unsigned char RegAddr, unsigned int len, char* cBuff)
{
	unsigned int i = 0;

	*pI2C = I2CAddr; //FIrst write which I2C module is being addressed. MPU or HMC
	*pI2CReg = RegAddr; // give the address of register from which you want to read in burst fashion
	*pI2CDataLen = len; // number of bytes you want to read
	*pI2CCmd = (I2C_READ_CMD_MASK | I2C_START_CMD_MASK); // give read+start command

	/*The CPU is free to do its work until the following flag is set. The HW module will do a burst access and read the number of requested bytes
	 * from the slow I2C slave and keep it onchip in a FIFO. Once the flag is set, the data can be read from the FIFO rapidly*/
	while((*pI2CDataAvailable != 0x1));

	for (i = 0; i < len; i++)
	{
		cBuff[i] = *pI2C;
	}

	return;
}
