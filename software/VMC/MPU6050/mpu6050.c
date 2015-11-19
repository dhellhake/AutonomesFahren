/*
 * mpu6050.c
 *
 *  Created on: 05.11.2015
 *      Author: Mexx
 */

#include "mpu6050.h"

unsigned char getDeviceID()
{
	char cBuff[MPU6050_WHO_AM_I_LENGTH];

	I2CRead(MPU_SLAVE_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_LENGTH, cBuff);
	return cBuff[0];
}
