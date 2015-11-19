/*************************************************************************
* Copyright (c) 2004 Altera Corporation, San Jose, California, USA.      *
* All rights reserved. All use of this software and documentation is     *
* subject to the License Agreement located at the end of this file below.*
**************************************************************************
* Description:                                                           *
* The following is a simple hello world program running MicroC/OS-II.The * 
* purpose of the design is to be a very simple application that just     *
* demonstrates MicroC/OS-II running on NIOS II.The design doesn't account*
* for issues such as checking system call return codes. etc.             *
*                                                                        *
* Requirements:                                                          *
*   -Supported Example Hardware Platforms                                *
*     Standard                                                           *
*     Full Featured                                                      *
*     Low Cost                                                           *
*   -Supported Development Boards                                        *
*     Nios II Development Board, Stratix II Edition                      *
*     Nios Development Board, Stratix Professional Edition               *
*     Nios Development Board, Stratix Edition                            *
*     Nios Development Board, Cyclone Edition                            *
*   -System Library Settings                                             *
*     RTOS Type - MicroC/OS-II                                           *
*     Periodic System Timer                                              *
*   -Know Issues                                                         *
*     If this design is run on the ISS, terminal output will take several*
*     minutes per iteration.                                             *
**************************************************************************/


#include "vmc.h"
#include "time.h"
#include "MPU6050/mpu6050.h"

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK    sensorCollector_stk[TASK_STACKSIZE];
OS_STK    drivingTask_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
/* Each Task must have a unique priority number
 * between 0 and 254. Priority level 0 is the
 * highest priority.
 */
#define TASK1_PRIORITY      1
#define TASK2_PRIORITY      2

/* Prints "Hello World" and sleeps for three seconds */
void sensorCollector(void* pdata)
{
	unsigned int i = 0;
			unsigned int uiGapStartEnc = 0;
			unsigned int uiSouthEastDis = 0;
			short AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, x, y, z;
			char cBuff[32];
			bool_t bGapStarted = false;
			ParkingStateType_t cs = Ready;
			*pwm_enable = 0;

	I2CWrite(MPU_SLAVE_ADDRESS, 0x6b, 0x0);
	I2CRead(MPU_SLAVE_ADDRESS, 0x75, 1, cBuff);
	printf("WhoAmI = %x\n", (unsigned int)cBuff[0]);
	while (1)
	{
		// HC_SR04
		printf("new\n");
		while (*pHc_sr04 != 0xff);

		//printf("2\n");
		*pHc_sr04 = 0xC7;

		//printf("3\n");
		while (*pHc_sr04 != 0xff);
		//delay(10000000);

		for (i = 0; i < NUMBER_OF_ULTRA_SOUND_DEVICES; i++)
		{

			printf("%u = %u\n",i, MeasureDistance(i));
		}
		printf("\n\n");
		//delay(10000000);
		//printf("4\n");;



			AcX = AcY = AcZ = Tmp = GyX = GyY = GyZ = 0;

			I2CRead(MPU_SLAVE_ADDRESS, 0x3B, 14, cBuff);

			AcX = (cBuff[0] << 8) | (cBuff[1] & 0xff);
			AcY = (cBuff[2] << 8) | (cBuff[3] & 0xff);
			AcZ = cBuff[4] << 8 | (cBuff[5] & 0xff);
			//AcZ = cBuff[5];

			Tmp = (cBuff[6] << 8) | (cBuff[7] & 0xff);

			GyX = (cBuff[8] << 8) | (cBuff[9] & 0xff);
			GyY = (cBuff[10] << 8) | (cBuff[11] & 0xff);
			GyZ = (cBuff[12] << 8) | (cBuff[13] & 0xff);

			printf("AcX = %d\n", AcX);
			printf("AcY = %d\n", AcY);
			printf("AcZ = %d\n", AcZ);
	//		printf("%u,%u\n", cBuff[4], (cBuff[5] & 0xff));

			printf("Tmp = %f\n", (float)Tmp/340 + 36.53);

			printf("GyX = %d\n", GyX);
			printf("GyY = %d\n", GyY);
			printf("GyZ = %d\n", GyZ);

			//delay(10000000);


	printf("Hello from task sensorCollector\n");
	OSTimeDlyHMSM(0, 0, 3, 0);
  }
}
/* Prints "Hello World" and sleeps for three seconds */
void drivingTask(void* pdata)
{
	unsigned int i = 10;

	OSTimeDlyHMSM(0, 0, 1, 0);

	/* With MicroC/OS-II, each task must be an infinite loop */
	while (1)
	{
		if(i>=100)
			i = 0;
		else
			i+=10;

		/* Task is suspended (will not run) for one complete second
		 * by calling OSTimeDlyHMSM(). The HMSM stands for Hours,
		 * Minutes, Seconds and Milliseconds and corresponds to the
		 * arguments passed to OSTimeDlyHMSM(). Because the Task is
		 * suspended for one second, MicroC/OS-II will start executing
		 * the next most important task. You should note that without
		 * OSTimeDlyHMSM() (or other similar functions), the Task would
		 * be a true infinite loop and other tasks would never get a
		 * chance to run.
		 */
		OSTimeDlyHMSM(0, 0, 1, 0);

		printf("PWM: %d", i);

		set_duty_cycle(pFrontRightDutySet, i);
		set_duty_cycle(pRearRightDutySet, i);
		set_duty_cycle(pRearLeftDutySet, i);
		set_duty_cycle(pFrontLeftDutySet, i);

		*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );

	}
}

/*
void task_15ms(void* pdata)
{
	while (1)
	{
		INT32U start_execution = OSTimeGet();

		INT8U devID = getDeviceID();
		printf("GyZ = %d\n", devID);



		INT32U timeToWait = 15 - (OSTimeGet() - start_execution);
		if(timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}*/

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  
  init();

  OSInit();

  /* Create Semaphor */
  //Sem = OSSemCreate(1);

  /* Acquiring semaphore is done by calling OSSemPend()
   * and passing it the 'handle' of the semaphore which
   * was created earlier. The second argument of OSSemPen()
   * is used to specify a timout. A value of 0 means that
   * this task will wait forever for the semaphore. If the
   * semaphore was 'owned' by another task, MicroC/OS-II would
   * have to suspend this task and execute the next most
   * important task.
   */
  //OSSemPend(Sem, timeout, &err);

  /* Semaphore is released by calling OSSemPost(). Here
   * simply the handle of the semaphore has to be specified.
   */
  //OSSemPost(Sem);

  /* Processor specific macro used to disable interrupts */
  //OS_ENTER_CRITICAL();
  //OS_EXIT_CRITICAL();

  /* Changing tick rate is handled by PC service called
   * PC_SetTickRate() and is passed the desired tick rate
   * (Set OS_TICKS_PER_SEC in system.h)
   */
  // PC_SetTickRate(OS_TICKS_PER_SEC)

  /* Clear Conext Switch Counter */
  //OSCtxSwCtr = 0;

  OSTaskCreateExt(sensorCollector,
                      NULL,
                      (void *)&sensorCollector_stk[TASK_STACKSIZE-1],
                      TASK1_PRIORITY,
                      TASK1_PRIORITY,
                      sensorCollector_stk,
                      TASK_STACKSIZE,
                      NULL,
                      0);
               
  OSTaskCreateExt(drivingTask,
                  NULL,
                  (void *)&drivingTask_stk[TASK_STACKSIZE-1],
                  TASK2_PRIORITY,
                  TASK2_PRIORITY,
                  drivingTask_stk,
                  TASK_STACKSIZE,
                  NULL,
                  0);

  OSStart();
  return 0;
}

/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2004 Altera Corporation, San Jose, California, USA.           *
* All rights reserved.                                                        *
*                                                                             *
* Permission is hereby granted, free of charge, to any person obtaining a     *
* copy of this software and associated documentation files (the "Software"),  *
* to deal in the Software without restriction, including without limitation   *
* the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
* and/or sell copies of the Software, and to permit persons to whom the       *
* Software is furnished to do so, subject to the following conditions:        *
*                                                                             *
* The above copyright notice and this permission notice shall be included in  *
* all copies or substantial portions of the Software.                         *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
* This agreement shall be governed in all respects by the laws of the State   *
* of California and by the laws of the United States of America.              *
* Altera does not recommend, suggest or require that this reference design    *
* file be used in conjunction or combination with any other product.          *
******************************************************************************/
