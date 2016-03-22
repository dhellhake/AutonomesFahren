/**
 * VCMlib.c
 *
 *  Created on: 07.01.2016
 *      Author: Dominik Hellhake (3662000)
 *
 *@file
 *@brief main file of vehicle management and control
 *
 */

#include "../VMC.h"

void initVMC(void)
{
	INT8U return_code = OS_NO_ERR;

	initUltrasoundSensors();

	pEmergencyStop = (INT8U*)(STATE_CMD_MEMORY_BASE | 0x04);

	mutex = OSMutexCreate(MUTEX_PRIORITY, &return_code);

	myMPU = getMPU();
	initMPU(myMPU);
	mpuDmpInitialize(myMPU);
	calibrateMPU(myMPU);
}

void MNV_InitQueue()
{
	//Get the main-Queue
	mnv_queue_t* mnv_queue = (mnv_queue_t *) MNV_QUEUE_BASE;

	mnv_queue->_read = 0x0;
	mnv_queue->_write = 0x0;
	mnv_queue->_buffer = (mnv_item_t *) (MNV_QUEUE_BASE + sizeof(mnv_queue_t));
}

/*
  Reads a Byte out of the receive-queue of USART3
  Post-increment read
*/
inline mnv_item_t* MNV_Queue_DeQueue()
{
  mnv_item_t* tmp = 0x0;

  //Get the main-Queue
  mnv_queue_t* mnv_queue = (mnv_queue_t *) MNV_QUEUE_BASE;

  //Read the current item
  tmp = (mnv_item_t *) (((unsigned int) mnv_queue->_buffer) + mnv_queue->_read);
  tmp->_dequeued = 1;

  //Increase read-pointer and wrap on overflow
  mnv_queue->_read = (mnv_queue->_read + sizeof(mnv_item_t)) & (MANUER_QUEUE_SPAN - 1);

  return tmp;
}

/*
 * Returns 1 if at least one mnv_item_t is available inside the queue
 */
inline unsigned int MNV_Queue_Item_Available()
{
	//Get the main-Queue
	mnv_queue_t* mnv_queue = (mnv_queue_t *) MNV_QUEUE_BASE;

	return mnv_queue->_read == mnv_queue->_write ? 0 : 1;
}

/*
 * Creates and enqueues a new mnv_item
 */
inline mnv_item_t* MNV_Queue_EnQueue(unsigned int type)
{
	//Get the main-Queue
	mnv_queue_t* mnv_queue = (mnv_queue_t *) MNV_QUEUE_BASE;

	mnv_item_t* new_item = (mnv_item_t *) (((unsigned int) mnv_queue->_buffer) + mnv_queue->_write);
	new_item->_dequeued = 0;
	new_item->_type = type;

	//Increase write-pointer and wrap on overflow
	mnv_queue->_write = (mnv_queue->_write + sizeof(mnv_item_t)) & (MANUER_QUEUE_SPAN - 1);

	return new_item;
}

void MNV_Queue_Test(void *pdata)
{
	MNV_InitQueue();

	mnv_item_t* testItem;

	unsigned x = 0;
	for(x = 0; x < 32; x++)
		testItem = MNV_Queue_EnQueue(x);


	while(MNV_Queue_Item_Available() == 1)
	{
		testItem = MNV_Queue_DeQueue();

		printf("%d \n", testItem->_type);
	}
}


snr_sonic_t* SONICGetState(enum SONIC_SENSOR_POS position)
{
	return (snr_sonic_t *) (SNR_SONIC_BASE + (sizeof(snr_sonic_t) * (int)position));
}

void SONICSetState(unsigned int distance, enum SONIC_SENSOR_POS position)
{
	snr_sonic_t* currentSonic = (snr_sonic_t *) (SNR_SONIC_BASE + (sizeof(snr_sonic_t) * (int)position));

	currentSonic->_distance = distance;
	currentSonic->_position = position;
	//currentSonic->_timestamp =
}

act_wheel_t* WHLGetState(enum WHEEL_POS position)
{
	return (act_wheel_t *) (ACT_WHL_BASE + (sizeof(act_wheel_t) * (int)position));
}
void WHLSetState(int ticks, int distance, enum WHEEL_POS position)
{
	act_wheel_t* currentWhl = (act_wheel_t *) (ACT_WHL_BASE + (sizeof(act_wheel_t) * (int)position));

	currentWhl->_distance = distance;
	currentWhl->_ticks = ticks;
	//currentWhl->_timestamp =
}


snr_dmp_t* DMPGetValueSet()
{
	return (snr_dmp_t *) SNR_DMP_BASE;
}
void DMPSetValueSet(float yaw, float pitch, float roll, int accX, int accY, int accZ)
{
	snr_dmp_t* currentSet = (snr_dmp_t *) SNR_DMP_BASE;

	currentSet->_yaw = yaw;
	currentSet->_pitch = pitch;
	currentSet->_roll = roll;
	currentSet->_accX = accX;
	currentSet->_accY = accY;
	currentSet->_accZ = accZ;
	//currentSet->_timestamp =
}
