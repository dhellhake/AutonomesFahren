/*
 * VCMlib.h
 *
 *  Created on: 07.01.2016
 *      Author: Dominik Hellhake (3662000)
 */

#ifndef VCMLIB_H_
#define VCMLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MNV_QUEUE_BASE (0x80000000 | MANUER_QUEUE_BASE)

typedef struct
{
	unsigned int _type;
	volatile unsigned int _dequeued;
} mnv_item_t;

typedef struct
{
	unsigned int _read;
	unsigned int _write;
	volatile mnv_item_t* _buffer;
} mnv_queue_t;

extern void MNV_IntQueue();
extern inline mnv_item_t* MNV_Queue_DeQueue();
extern inline mnv_item_t* MNV_Queue_EnQueue(unsigned int type);
extern inline unsigned int MNV_Queue_Item_Available();
extern void MNV_Queue_Test(void *pdata);
extern void speedControl(void *pdata);
extern void uart(void *pdata);
extern INT16S calcSteeringOffset(INT16S steeringValue);
//char getMeanSensorDistance(unsigned int *means);
char getMeanSensorDistance(unsigned int *means, INT8U sensorIndex);
void makeMeasurement();
void startMeasurement(INT8U sensorIndex);
void initUltrasoundSensors();
void initVMC(void);

volatile unsigned int *pEmergencyStop;

#define NUMBER_MEAN_VALUES 4
#define ERROR_TOLERANCE 1000

typedef struct SensorValue
{
	unsigned int ultraSoundValues[NUMBER_MEAN_VALUES];
	int counter;
}SENSOR_VALUE;

SENSOR_VALUE values[NUMBER_OF_ULTRA_SOUND_DEVICES];

#ifdef __cplusplus
}
#endif

#endif /* VCMLIB_H_ */
