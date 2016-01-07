/*
 * VCMlib.c
 *
 *  Created on: 07.01.2016
 *      Author: Dominik Hellhake (3662000)
 */

#include "VMClib.h"

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
