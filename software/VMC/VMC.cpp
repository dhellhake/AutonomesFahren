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
#define __cplusplus
#define DEBUG
#define DEBUG_SPD_CTRL
#define TEST



#include "VMC.h"

using namespace std;

INT32S desired_speed = 500;
INT16S Kp_SpeedCtrl_num = 296;
INT16S Kp_SpeedCtrl_den = 1000;
INT16S Ki_SpeedCtrl_num = 897;
INT16S Ki_SpeedCtrl_den = 1000;
INT16S Kd_SpeedCtrl_num = 0;
INT16S Kd_SpeedCtrl_den = 1000;
INT32S I_SpeedCtrl_min = -100;//-10000;
INT32S I_SpeedCtrl_max = 100;//10000;
INT32S P_SpeedCtrl = 0;
INT32S I_SpeedCtrl = 0;
INT32S D_SpeedCtrl = 0;
INT16S g_i16s_PWMSpeedCtrl = 0;
INT16S Fast_Forward_Control = 0;
INT32S I_SpeedCtrl_error = 0;
INT32U speed = 0;
INT32S e_speed = 0;
INT32S e_speed_old = 0;
INT16S PWM_SpeedCtrl_max = 80;	// more than 80% PWM-DutyCycle leads to faulty values of the wheel encoders
INT16S PWM_SpeedCtrl_min = 0;
INT32S step_size = 50;

#ifndef TEST

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK    sensorCollector_stk[TASK_STACKSIZE];
OS_STK    drivingTask_stk[TASK_STACKSIZE];
OS_STK	  speedControl_stk[TASK_STACKSIZE];
OS_STK	  uart_stk[TASK_STACKSIZE];
OS_STK	  test_stk[TASK_STACKSIZE];
OS_STK    step_response_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
/* Each Task must have a unique priority number
 * between 0 and 254. Priority level 0 is the
 * highest priority.
 */
#define TASK_SPD_CTRL_PRIORITY		0
#define TASK1_PRIORITY      		2
#define TASK2_PRIORITY      		3
#define TASK_UART_PRIORITY			4
#define TASK_TEST_PRIORITY			5
#define TASK_STP_RESP_PRIORITY		1

/* Prints "Hello World" and sleeps for three seconds */
void sensorCollector(void* pdata)
{
	unsigned int i = 0;
			unsigned int uiGapStartEnc = 0;
			unsigned int uiSouthEastDis = 0;
			short AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, x, y, z;
			char cBuff[32];
			bool bGapStarted = false;
			//ParkingStateType_t cs = Ready;
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

void test(void* pdata)
{
	int i = 0;
	g_i16s_PWMSpeedCtrl = 40;
	*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );

	calcSteeringOffset(33);
	OSTimeDlyHMSM(0,0,3,0);

	calcSteeringOffset(-33);
	OSTimeDlyHMSM(0,0,3,0);

	calcSteeringOffset(66);
	OSTimeDlyHMSM(0,0,3,0);

	calcSteeringOffset(-66);
	OSTimeDlyHMSM(0,0,3,0);

	calcSteeringOffset(100);
	OSTimeDlyHMSM(0,0,3,0);

	calcSteeringOffset(-100);
	OSTimeDlyHMSM(0,0,3,0);

	/*for(i = 0; i < 20; i++)
	{
		desired_speed = i * 100;
#ifdef DEBUG
		printf("desired_speed: %i\n", desired_speed);
#endif
		OSTimeDlyHMSM(0,0,1,0);
	}

	for(i = 20; i >= 0; i--)
	{
		desired_speed = i * 100;
#ifdef DEBUG
		printf("desired_speed: %i\n", desired_speed);
#endif
		OSTimeDlyHMSM(0,0,1,0);
	}*/
}






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

 /* OSTaskCreateExt(sensorCollector,
                      NULL,
                      (void *)&sensorCollector_stk[TASK_STACKSIZE-1],
                      TASK1_PRIORITY,
                      TASK1_PRIORITY,
                      sensorCollector_stk,
                      TASK_STACKSIZE,
                      NULL,
                      0);*/
               
   /*OSTaskCreateExt(speedControl,
                  NULL,
                  &speedControl_stk[TASK_STACKSIZE-1],
                  TASK_SPD_CTRL_PRIORITY,
                  TASK_SPD_CTRL_PRIORITY,
                  speedControl_stk,
                  TASK_STACKSIZE,
                  NULL,
                  0);*/

  /*OSTaskCreateExt(step_response,
                    NULL,
                    &step_response_stk[TASK_STACKSIZE-1],
                    TASK_STP_RESP_PRIORITY,
                    TASK_STP_RESP_PRIORITY,
                    step_response_stk,
                    TASK_STACKSIZE,
                    NULL,
                    0);*/

  /*OSTaskCreateExt(step_response,
                    NULL,
                    &test_stk[TASK_STACKSIZE-1],
                    TASK_TEST_PRIORITY,
                    TASK_TEST_PRIORITY,
                    test_stk,
                    TASK_STACKSIZE,
                    NULL,
                    0);*/

  /*OSTaskCreateExt(uart,
                    NULL,
                    &uart_stk[TASK_STACKSIZE-1],
                    TASK_UART_PRIORITY,
                    TASK_UART_PRIORITY,
                    uart_stk,
                    TASK_STACKSIZE,
                    NULL,
                    0);*/

  OSTaskCreateExt(test,
                    NULL,
                    &test_stk[TASK_STACKSIZE-1],
                    TASK_TEST_PRIORITY,
                    TASK_TEST_PRIORITY,
                    test_stk,
                    TASK_STACKSIZE,
                    NULL,
                    0);

  OSStart();
  return 0;
}

#endif

#ifdef TEST

//#define MOTOR_TEST
#define HC_SR04_TEST
//#define MPU_TEST
//#define COMPASS_TEST

#ifdef __cplusplus
typedef bool bool_t;
#else
typedef enum
{
	true,
	false
} bool_t;
#endif

int main()
{
	init();
	unsigned int i = 0;
	unsigned int uiGapStartEnc = 0;
	unsigned int uiSouthEastDis = 0;
	short AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, x, y, z;
	char cBuff[32];
	bool_t bGapStarted = false;
	ParkingStateType_t cs = Ready;
	*pwm_enable = 0;

	printf("s\n");

#ifdef MOTOR_TEST
	volatile unsigned int fr, rr, rl, fl;
	set_duty_cycle(pFrontRightDutySet, 50);
	set_duty_cycle(pRearRightDutySet, 50);
	set_duty_cycle(pRearLeftDutySet, 50);
	set_duty_cycle(pFrontLeftDutySet, 50);

	*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );
	delay(10000000);
    *pwm_enable = PAUSE_ENC_MASK;

	fr = *pFrontRightEncRead;
	rr = *pRearRightEncRead;
	rl = *pRearLeftEncRead;
	fl = *pFrontLeftEncRead;
	*pwm_enable = 0;

	printf("Front right = %u\n", fr);
	printf("rear right = %u\n", rr);
	printf("rear left = %u\n", rl);
	printf("Front left = %u\n", fl);

	printf("Restarting the same number of rotations\n");

	delay(100000);


	*pFrontRightEncSet = fr;
	*pRearRightEncSet = rr;
	*pRearLeftEncSet = rl;
	*pFrontLeftEncSet = fl;

	*pwm_enable = (ALL_WHEEL_FWD_MASK | PLAY_BACK_MASK | ENABLE_ENC_MASK );

	while (!(*pwm_enable & WHEEL_READY_MASK));
	fr = *pFrontRightEncRead;
	rr = *pRearRightEncRead;
	rl = *pRearLeftEncRead;
	fl = *pFrontLeftEncRead;
	*pwm_enable = 0;

	printf("Front right = %u\n", fr);
	printf("rear right = %u\n", rr);
	printf("rear left = %u\n", rl);
	printf("Front left = %u\n", fl);
#endif

#ifdef HC_SR04_TEST
	while (1)
	{
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
		delay(10000000);
		//printf("4\n");;
}
#endif

#ifdef MPU_TEST
	I2CWrite(MPU_SLAVE_ADDRESS, 0x6b, 0x0);
	I2CRead(MPU_SLAVE_ADDRESS, 0x75, 1, cBuff);
	printf("WhoAmI = %x\n", (unsigned int)cBuff[0]);

	while(1)
	{
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

		delay(10000000);
	}

#endif

#ifdef COMPASS_TEST

	I2CWrite(HMC5883L_SLAVE_ADDRESS, 0x02, 00); //set contineous read mode

	while(1)
	{
		I2CRead(HMC5883L_SLAVE_ADDRESS, 0x03, 6, cBuff);

		x = (cBuff[0] << 8) | (cBuff[1] & 0xff);
		z = (cBuff[2] << 8) | (cBuff[3] & 0xff);
		y = (cBuff[4] << 8) | (cBuff[5] & 0xff);

		printf("X = %d\n", x);
		printf("Y = %d\n", y);
		printf("Z = %d\n", z);

		delay(10000000);
	}

#endif

	return 0;
}


#endif
