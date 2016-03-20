/*
 * helper_3dmathc.h
 *
 *  Created on: 07.01.2016
 *      Author: spret
 */

#ifndef HELPER_3DMATHC_H_
#define HELPER_3DMATHC_H_

typedef struct QuaternionStruct {
	float w;
	float x;
	float y;
	float z;
} CQuaternion;

typedef struct VectorInt16Struct {
	short x;
	short y;
	short z;
} CVectorInt16;

typedef struct VectorFloatStruct {
	float x;
	float y;
	float z;
} CVectorFloat;

#endif /* HELPER_3DMATHC_H_ */
