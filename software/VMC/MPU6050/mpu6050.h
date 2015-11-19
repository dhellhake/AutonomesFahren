/*
 * mpu6050.h
 *
 *  Created on: 05.11.2015
 *      Author: Mexx
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "../head.h"

#define MPU6050_RA_WHO_AM_I 		0x75
#define MPU6050_WHO_AM_I_LENGTH 	1


unsigned char getDeviceID();


#endif /* MPU6050_H_ */
