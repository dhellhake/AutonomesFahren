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
//#define TEST

#ifndef TEST

#include "VMC.h"
#include "time.h"
#include "MPU6050/mpu6050.h"
#include <iostream>
#include <cstdlib>
#include "altera_up_avalon_rs232.h"
#include "head.c"

using namespace std;

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

void uart(void *pdata)
{
	int i = 0;
	alt_u8 testStr[] = "Hello\n1";
	alt_u8 parityErr = 0;
	alt_up_rs232_dev *pUart;

	while (1)
	  {
	    pUart = alt_up_rs232_open_dev("/dev/rs232_0");

	    printf("%x\n", pUart->base);
	    for (i = 0; i < 7; i++)
	    {
	        alt_u8 c;
	        alt_up_rs232_write_data(pUart, testStr[i]); // externally looped back
	    }
	    OSTimeDlyHMSM(0, 0, 1, 0);

	    /*for (i = 0; i < 7; )
	        {
	            alt_u8 c;
	            if (alt_up_rs232_read_data(pUart, &c, &parityErr) == 0)
	            {
	                printf("%c", c);
	                i++;
	            }
	        }*/
	  }
}

/* Task for Speed Control with PID-Regulator */
void speedControl(void *pdata)
{
	int time = 0;
	int time_old = 0;
	int delta_t = 0;

	INT32U whl_ticks_fl = 0;
	INT32U whl_ticks_fr = 0;
	INT32U whl_ticks_rl = 0;
	INT32U whl_ticks_rr = 0;

	INT32U whl_ticks_fl_old = 0;
	INT32U whl_ticks_fr_old = 0;
	INT32U whl_ticks_rl_old = 0;
	INT32U whl_ticks_rr_old = 0;

	INT32U delta_whl_ticks_fl = 0;
	INT32U delta_whl_ticks_fr = 0;
	INT32U delta_whl_ticks_rl = 0;
	INT32U delta_whl_ticks_rr = 0;

	INT32U delta_s_fl = 0;
	INT32U delta_s_fr = 0;
	INT32U delta_s_rl = 0;
	INT32U delta_s_rr = 0;

	int actual_speed_fl = 0;
	int actual_speed_fr = 0;
	int actual_speed_rl = 0;
	int actual_speed_rr = 0;

	static int speed_fl[4] = {0, 0, 0, 0};
	static int speed_fr[4] = {0, 0, 0, 0};
	static int speed_rl[4] = {0, 0, 0, 0};
	static int speed_rr[4] = {0, 0, 0, 0};

	int i = 0;

	INT32U start_execution = 0;
	INT32U timeToWait = 0;

	// FIR-Filter leads to attack-time of regulator

	while(1)
	{
		start_execution = OSTimeGet();

		time_old = time;
		/* Save actual time in [ms] */
		time = OSTimeGet() / OS_TICKS_PER_MSEC;
		delta_t = time - time_old;

		whl_ticks_fl_old = whl_ticks_fl;
		whl_ticks_fr_old = whl_ticks_fr;
		whl_ticks_rl_old = whl_ticks_rl;
		whl_ticks_rr_old = whl_ticks_rr;

		// compensate wheel tick error at beginning
		if(time < 200)
		{
			whl_ticks_fl = 0;
			whl_ticks_fr = 0;
			whl_ticks_rl = 0;
			whl_ticks_rr = 0;
		}
		else
		{
			whl_ticks_fl = *pFrontLeftEncRead;
			whl_ticks_fr = *pFrontRightEncRead;
			whl_ticks_rl = *pRearLeftEncRead;
			whl_ticks_rr = *pRearRightEncRead;
		}

		delta_whl_ticks_fl = whl_ticks_fl - whl_ticks_fl_old;
		delta_whl_ticks_fr = whl_ticks_fr - whl_ticks_fr_old;
		delta_whl_ticks_rl = whl_ticks_rl - whl_ticks_rl_old;
		delta_whl_ticks_rr = whl_ticks_rr - whl_ticks_rr_old;

		delta_s_fl = delta_whl_ticks_fl * DISTANCE_PER_WHL_TICK;
		delta_s_fr = delta_whl_ticks_fr * DISTANCE_PER_WHL_TICK;
		delta_s_rl = delta_whl_ticks_rl * DISTANCE_PER_WHL_TICK;
		delta_s_rr = delta_whl_ticks_rr * DISTANCE_PER_WHL_TICK;

		if(i > 3)
		{
			i = 0;
		}

		speed_fl[i] = delta_s_fl*1000/delta_t; // [mm/s]
		speed_fr[i] = delta_s_fr*1000/delta_t;
		speed_rl[i] = delta_s_rl*1000/delta_t;
		speed_rr[i] = delta_s_rr*1000/delta_t;

		actual_speed_fl = (speed_fl[0] + speed_fl[1] + speed_fl[2] + speed_fl[3]) >> 2;
		actual_speed_fr = (speed_fr[0] + speed_fr[1] + speed_fr[2] + speed_fr[3]) >> 2;
		actual_speed_rl = (speed_rl[0] + speed_rl[1] + speed_rl[2] + speed_rl[3]) >> 2;
		actual_speed_rr = (speed_rr[0] + speed_rr[1] + speed_rr[2] + speed_rr[3]) >> 2;

		speed = ((actual_speed_rl + actual_speed_rr) >> 1);

		e_speed = desired_speed - speed;

		P_SpeedCtrl = ((Kp_SpeedCtrl_num * e_speed) / Kp_SpeedCtrl_den);

		I_SpeedCtrl_error = I_SpeedCtrl_error + e_speed;

		if(I_SpeedCtrl_error < I_SpeedCtrl_min)
		{
			I_SpeedCtrl_error = I_SpeedCtrl_min;
		}
		if(I_SpeedCtrl_error > I_SpeedCtrl_max)
		{
			I_SpeedCtrl_error = I_SpeedCtrl_max;
		}

		//TODO: Check I and D
		I_SpeedCtrl = ((Ki_SpeedCtrl_num * I_SpeedCtrl_error) / Ki_SpeedCtrl_den);

		D_SpeedCtrl = ((Kd_SpeedCtrl_num / SPD_CTRL_CYCLE_TIME_MS * (e_speed - e_speed_old)) / Kd_SpeedCtrl_den);

		Fast_Forward_Control = ((36 * desired_speed)/1000) + 18;

		PWM_SpeedCtrl = (INT16S) ((P_SpeedCtrl + I_SpeedCtrl + D_SpeedCtrl) + Fast_Forward_Control);

		if(PWM_SpeedCtrl < PWM_SpeedCtrl_min)
		{
			PWM_SpeedCtrl = PWM_SpeedCtrl_min;
		}
		else if(PWM_SpeedCtrl > PWM_SpeedCtrl_max)
		{
			PWM_SpeedCtrl = PWM_SpeedCtrl_max;
		}

		set_duty_cycle( pFrontRightDutySet, PWM_SpeedCtrl);
		set_duty_cycle(pRearRightDutySet, PWM_SpeedCtrl);
		set_duty_cycle(pRearLeftDutySet, PWM_SpeedCtrl);
		set_duty_cycle(pFrontLeftDutySet, PWM_SpeedCtrl);

		*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );

#ifdef DEBUG_SPD_CTRL
		printf("time: %d\n", time);
		printf("time_old: %d\n", time_old);
		printf("delta_t: %d\n", delta_t);

		printf("delta_whl_ticks_fl: %d\n", delta_whl_ticks_fl);
		printf("delta_whl_ticks_fr: %d\n", delta_whl_ticks_fr);
		printf("delta_whl_ticks_rl: %d\n", delta_whl_ticks_rl);
		printf("delta_whl_ticks_rr: %d\n", delta_whl_ticks_rr);

		printf("P_SpeedCtrl: %d\n", P_SpeedCtrl);
		printf("I_SpeedCtrl: %d\n", I_SpeedCtrl);
		printf("D_SpeedCtrl: %d\n", D_SpeedCtrl);
		printf("Fast_Forward_Control: %d\n", Fast_Forward_Control);
		printf("PWM_SpeedCtrl: %d\n", PWM_SpeedCtrl);

		printf("actual_speed_fl: %d\n", actual_speed_fl);
		printf("actual_speed_fr: %d\n", actual_speed_fr);
		printf("actual_speed_rl: %d\n", actual_speed_rl);
		printf("actual_speed_rr: %d\n", actual_speed_rr);

		printf("speed: %d\n", speed);
		printf("actual_speed_rr: %d\n", actual_speed_rr);
		printf("desired_speed: %d\n", desired_speed);
#endif

		i = i + 1;

		timeToWait = SPD_CTRL_CYCLE_TIME_MS - (OSTimeGet() - start_execution);

		if(timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}

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
/* Prints "Hello World" and sleeps for three seconds */
void drivingTask(void* pdata)
{
	int time = 0;
	int time_old = 0;
	int delta_t = 0;

	INT32U whl_ticks_fl = 0;
	INT32U whl_ticks_fr = 0;
	INT32U whl_ticks_rl = 0;
	INT32U whl_ticks_rr = 0;

	INT32U whl_ticks_fl_old = 0;
	INT32U whl_ticks_fr_old = 0;
	INT32U whl_ticks_rl_old = 0;
	INT32U whl_ticks_rr_old = 0;

	INT32U delta_whl_ticks_fl = 0;
	INT32U delta_whl_ticks_fr = 0;
	INT32U delta_whl_ticks_rl = 0;
	INT32U delta_whl_ticks_rr = 0;

	INT32U delta_s_fl = 0;
	INT32U delta_s_fr = 0;
	INT32U delta_s_rl = 0;
	INT32U delta_s_rr = 0;

	int actual_speed_fl = 0;
	int actual_speed_fr = 0;
	int actual_speed_rl = 0;
	int actual_speed_rr = 0;

	static int speed_fl[4] = {0, 0, 0, 0};
	static int speed_fr[4] = {0, 0, 0, 0};
	static int speed_rl[4] = {0, 0, 0, 0};
	static int speed_rr[4] = {0, 0, 0, 0};

	int i = 0;

	int k = 0;

	INT32U start_execution = 0;
	INT32U timeToWait = 0;



	unsigned int j = 0;

	OSTimeDlyHMSM(0, 0, 1, 0);

	/* With MicroC/OS-II, each task must be an infinite loop */
	while (1)
	{
		start_execution = OSTimeGet();

		time_old = time;
		/* Save actual time in [ms] */
		time = OSTimeGet() / OS_TICKS_PER_MSEC;
		delta_t = time - time_old;

		whl_ticks_fl_old = whl_ticks_fl;
		whl_ticks_fr_old = whl_ticks_fr;
		whl_ticks_rl_old = whl_ticks_rl;
		whl_ticks_rr_old = whl_ticks_rr;

		whl_ticks_fl = *pFrontLeftEncRead;
		whl_ticks_fr = *pFrontRightEncRead;
		whl_ticks_rl = *pRearLeftEncRead;
		whl_ticks_rr = *pRearRightEncRead;

		delta_whl_ticks_fl = whl_ticks_fl - whl_ticks_fl_old;
		delta_whl_ticks_fr = whl_ticks_fr - whl_ticks_fr_old;
		delta_whl_ticks_rl = whl_ticks_rl - whl_ticks_rl_old;
		delta_whl_ticks_rr = whl_ticks_rr - whl_ticks_rr_old;

		delta_s_fl = delta_whl_ticks_fl * DISTANCE_PER_WHL_TICK;
		delta_s_fr = delta_whl_ticks_fr * DISTANCE_PER_WHL_TICK;
		delta_s_rl = delta_whl_ticks_rl * DISTANCE_PER_WHL_TICK;
		delta_s_rr = delta_whl_ticks_rr * DISTANCE_PER_WHL_TICK;

		if(i > 3)
		{
			i = 0;
		}

		speed_fl[i] = delta_s_fl*1000/delta_t; // [mm/s]
		speed_fr[i] = delta_s_fr*1000/delta_t;
		speed_rl[i] = delta_s_rl*1000/delta_t;
		speed_rr[i] = delta_s_rr*1000/delta_t;

		actual_speed_fl = (speed_fl[0] + speed_fl[1] + speed_fl[2] + speed_fl[3]) >> 2;
		actual_speed_fr = (speed_fr[0] + speed_fr[1] + speed_fr[2] + speed_fr[3]) >> 2;
		actual_speed_rl = (speed_rl[0] + speed_rl[1] + speed_rl[2] + speed_rl[3]) >> 2;
		actual_speed_rr = (speed_rr[0] + speed_rr[1] + speed_rr[2] + speed_rr[3]) >> 2;

		speed = ((actual_speed_rl + actual_speed_rr) >> 1);

		set_duty_cycle( pFrontRightDutySet, j);
		set_duty_cycle(pRearRightDutySet, j);
		set_duty_cycle(pRearLeftDutySet, j);
		set_duty_cycle(pFrontLeftDutySet, j);

		if ((k%30) == 0)
		{
			if(j>=100)
				j = 0;
			else
				j+=5;
		}

		k = k + 1;
		i = i + 1;

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
		//OSTimeDlyHMSM(0, 0, 1, 0);

		timeToWait = SPD_CTRL_CYCLE_TIME_MS - (OSTimeGet() - start_execution);

		if(timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);

		/*set_duty_cycle(pFrontRightDutySet, i);
		set_duty_cycle(pRearRightDutySet, i);
		set_duty_cycle(pRearLeftDutySet, i);
		set_duty_cycle(pFrontLeftDutySet, i);*/

		*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );
		//*pwm_enable = (TURN_LEFT_MASK | ENABLE_ENC_MASK );
#ifdef DEBUG
		printf("-----------------------------------\n");
		printf("PWM: %d\n", j);
		printf("-----------------------------------\n");
		printf("time: %d\n", time);
		printf("time_old: %d\n", time_old);
		printf("delta_t: %d\n", delta_t);

		printf("delta_whl_ticks_fl: %d\n", delta_whl_ticks_fl);
		printf("delta_whl_ticks_fr: %d\n", delta_whl_ticks_fr);
		printf("delta_whl_ticks_rl: %d\n", delta_whl_ticks_rl);
		printf("delta_whl_ticks_rr: %d\n", delta_whl_ticks_rr);

		printf("actual_speed_fl: %d\n", actual_speed_fl);
		printf("actual_speed_fr: %d\n", actual_speed_fr);
		printf("actual_speed_rl: %d\n", actual_speed_rl);
		printf("actual_speed_rr: %d\n", actual_speed_rr);

		printf("speed: %d\n", speed);
#endif

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

	for(i = 0; i < 20; i++)
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
	}
}


void step_response(void *pdata)
{
	int time = 0;
		int time_old = 0;
		int delta_t = 0;

		INT32U whl_ticks_fl = 0;
		INT32U whl_ticks_fr = 0;
		INT32U whl_ticks_rl = 0;
		INT32U whl_ticks_rr = 0;

		INT32U whl_ticks_fl_old = 0;
		INT32U whl_ticks_fr_old = 0;
		INT32U whl_ticks_rl_old = 0;
		INT32U whl_ticks_rr_old = 0;

		INT32U delta_whl_ticks_fl = 0;
		INT32U delta_whl_ticks_fr = 0;
		INT32U delta_whl_ticks_rl = 0;
		INT32U delta_whl_ticks_rr = 0;

		INT32U delta_s_fl = 0;
		INT32U delta_s_fr = 0;
		INT32U delta_s_rl = 0;
		INT32U delta_s_rr = 0;

		int actual_speed_fl = 0;
		int actual_speed_fr = 0;
		int actual_speed_rl = 0;
		int actual_speed_rr = 0;

		static int speed_fl[4] = {0, 0, 0, 0};
		static int speed_fr[4] = {0, 0, 0, 0};
		static int speed_rl[4] = {0, 0, 0, 0};
		static int speed_rr[4] = {0, 0, 0, 0};

		int i = 0;

		INT32U start_execution = 0;
		INT32U timeToWait = 0;

		set_duty_cycle( pFrontRightDutySet, 0);
		set_duty_cycle(pRearRightDutySet, 0);
		set_duty_cycle(pRearLeftDutySet, 0);
		set_duty_cycle(pFrontLeftDutySet, 0);

		OSTimeDlyHMSM(0, 0, 5, 0);
		// FIR-Filter leads to attack-time of regulator

		while(1)
		{
			start_execution = OSTimeGet();

			time_old = time;
			/* Save actual time in [ms] */
			time = OSTimeGet() / OS_TICKS_PER_MSEC;
			delta_t = time - time_old;

			whl_ticks_fl_old = whl_ticks_fl;
			whl_ticks_fr_old = whl_ticks_fr;
			whl_ticks_rl_old = whl_ticks_rl;
			whl_ticks_rr_old = whl_ticks_rr;

			whl_ticks_fl = *pFrontLeftEncRead;
			whl_ticks_fr = *pFrontRightEncRead;
			whl_ticks_rl = *pRearLeftEncRead;
			whl_ticks_rr = *pRearRightEncRead;

			delta_whl_ticks_fl = whl_ticks_fl - whl_ticks_fl_old;
			delta_whl_ticks_fr = whl_ticks_fr - whl_ticks_fr_old;
			delta_whl_ticks_rl = whl_ticks_rl - whl_ticks_rl_old;
			delta_whl_ticks_rr = whl_ticks_rr - whl_ticks_rr_old;

			delta_s_fl = delta_whl_ticks_fl * DISTANCE_PER_WHL_TICK;
			delta_s_fr = delta_whl_ticks_fr * DISTANCE_PER_WHL_TICK;
			delta_s_rl = delta_whl_ticks_rl * DISTANCE_PER_WHL_TICK;
			delta_s_rr = delta_whl_ticks_rr * DISTANCE_PER_WHL_TICK;

			if(i > 3)
			{
				i = 0;
			}

			speed_fl[i] = delta_s_fl*1000/delta_t; // [mm/s]
			speed_fr[i] = delta_s_fr*1000/delta_t;
			speed_rl[i] = delta_s_rl*1000/delta_t;
			speed_rr[i] = delta_s_rr*1000/delta_t;

			actual_speed_fl = (speed_fl[0] + speed_fl[1] + speed_fl[2] + speed_fl[3]) >> 2;
			actual_speed_fr = (speed_fr[0] + speed_fr[1] + speed_fr[2] + speed_fr[3]) >> 2;
			actual_speed_rl = (speed_rl[0] + speed_rl[1] + speed_rl[2] + speed_rl[3]) >> 2;
			actual_speed_rr = (speed_rr[0] + speed_rr[1] + speed_rr[2] + speed_rr[3]) >> 2;

			speed = ((actual_speed_rl + actual_speed_rr) >> 1);

			set_duty_cycle( pFrontRightDutySet, step_size);
			set_duty_cycle(pRearRightDutySet, step_size);
			set_duty_cycle(pRearLeftDutySet, step_size);
			set_duty_cycle(pFrontLeftDutySet, step_size);

			*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );

	#ifdef DEBUG
			printf("%d;%d\n", time, speed);
			/*printf("time: %d\n", time);
			printf("time_old: %d\n", time_old);
			printf("delta_t: %d\n", delta_t);

			printf("delta_whl_ticks_fl: %d\n", delta_whl_ticks_fl);
			printf("delta_whl_ticks_fr: %d\n", delta_whl_ticks_fr);
			printf("delta_whl_ticks_rl: %d\n", delta_whl_ticks_rl);
			printf("delta_whl_ticks_rr: %d\n", delta_whl_ticks_rr);

			printf("PWM_SpeedCtrl: %d\n", PWM_SpeedCtrl);

			printf("actual_speed_fl: %d\n", actual_speed_fl);
			printf("actual_speed_fr: %d\n", actual_speed_fr);
			printf("actual_speed_rl: %d\n", actual_speed_rl);
			printf("actual_speed_rr: %d\n", actual_speed_rr);

			printf("speed: %d\n", speed);
			printf("actual_speed_rr: %d\n", actual_speed_rr);
			printf("desired_speed: %d\n", desired_speed);*/
	#endif

			i = i + 1;

			timeToWait = STP_RESP_CYCLE_TIME_MS - (OSTimeGet() - start_execution);

			if(timeToWait > 0)
				OSTimeDlyHMSM(0, 0, 0, timeToWait);
		}
}

void fast_forward_ctrl(void *pdata)
{
		int PWM = 0;
		int ctr = 0;

		int time = 0;
			int time_old = 0;
			int delta_t = 0;

			INT32U whl_ticks_fl = 0;
			INT32U whl_ticks_fr = 0;
			INT32U whl_ticks_rl = 0;
			INT32U whl_ticks_rr = 0;

			INT32U whl_ticks_fl_old = 0;
			INT32U whl_ticks_fr_old = 0;
			INT32U whl_ticks_rl_old = 0;
			INT32U whl_ticks_rr_old = 0;

			INT32U delta_whl_ticks_fl = 0;
			INT32U delta_whl_ticks_fr = 0;
			INT32U delta_whl_ticks_rl = 0;
			INT32U delta_whl_ticks_rr = 0;

			INT32U delta_s_fl = 0;
			INT32U delta_s_fr = 0;
			INT32U delta_s_rl = 0;
			INT32U delta_s_rr = 0;

			int actual_speed_fl = 0;
			int actual_speed_fr = 0;
			int actual_speed_rl = 0;
			int actual_speed_rr = 0;

			static int speed_fl[4] = {0, 0, 0, 0};
			static int speed_fr[4] = {0, 0, 0, 0};
			static int speed_rl[4] = {0, 0, 0, 0};
			static int speed_rr[4] = {0, 0, 0, 0};

			int i = 0;

			INT32U start_execution = 0;
			INT32U timeToWait = 0;

			set_duty_cycle( pFrontRightDutySet, 0);
			set_duty_cycle(pRearRightDutySet, 0);
			set_duty_cycle(pRearLeftDutySet, 0);
			set_duty_cycle(pFrontLeftDutySet, 0);

			OSTimeDlyHMSM(0, 0, 5, 0);
			// FIR-Filter leads to attack-time of regulator

			while(1)
			{
				start_execution = OSTimeGet();

				time_old = time;
				/* Save actual time in [ms] */
				time = OSTimeGet() / OS_TICKS_PER_MSEC;
				delta_t = time - time_old;

				whl_ticks_fl_old = whl_ticks_fl;
				whl_ticks_fr_old = whl_ticks_fr;
				whl_ticks_rl_old = whl_ticks_rl;
				whl_ticks_rr_old = whl_ticks_rr;

				whl_ticks_fl = *pFrontLeftEncRead;
				whl_ticks_fr = *pFrontRightEncRead;
				whl_ticks_rl = *pRearLeftEncRead;
				whl_ticks_rr = *pRearRightEncRead;

				delta_whl_ticks_fl = whl_ticks_fl - whl_ticks_fl_old;
				delta_whl_ticks_fr = whl_ticks_fr - whl_ticks_fr_old;
				delta_whl_ticks_rl = whl_ticks_rl - whl_ticks_rl_old;
				delta_whl_ticks_rr = whl_ticks_rr - whl_ticks_rr_old;

				delta_s_fl = delta_whl_ticks_fl * DISTANCE_PER_WHL_TICK;
				delta_s_fr = delta_whl_ticks_fr * DISTANCE_PER_WHL_TICK;
				delta_s_rl = delta_whl_ticks_rl * DISTANCE_PER_WHL_TICK;
				delta_s_rr = delta_whl_ticks_rr * DISTANCE_PER_WHL_TICK;

				if(i > 3)
				{
					i = 0;
				}

				speed_fl[i] = delta_s_fl*1000/delta_t; // [mm/s]
				speed_fr[i] = delta_s_fr*1000/delta_t;
				speed_rl[i] = delta_s_rl*1000/delta_t;
				speed_rr[i] = delta_s_rr*1000/delta_t;

				actual_speed_fl = (speed_fl[0] + speed_fl[1] + speed_fl[2] + speed_fl[3]) >> 2;
				actual_speed_fr = (speed_fr[0] + speed_fr[1] + speed_fr[2] + speed_fr[3]) >> 2;
				actual_speed_rl = (speed_rl[0] + speed_rl[1] + speed_rl[2] + speed_rl[3]) >> 2;
				actual_speed_rr = (speed_rr[0] + speed_rr[1] + speed_rr[2] + speed_rr[3]) >> 2;

				speed = ((actual_speed_rl + actual_speed_rr) >> 1);

				set_duty_cycle( pFrontRightDutySet, PWM);
				set_duty_cycle(pRearRightDutySet, PWM);
				set_duty_cycle(pRearLeftDutySet, PWM);
				set_duty_cycle(pFrontLeftDutySet, PWM);

				*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK );

		#ifdef DEBUG
				printf("%d;%d\n", PWM, speed);
		#endif
				if((ctr%30) == 0)
				{
					if(PWM >= PWM_SpeedCtrl_max)
					{
						PWM = 0;
					}
					else
					{
						PWM = PWM + 5;
					}
				}

				i = i + 1;

				ctr = ctr + 1;

				timeToWait = STP_RESP_CYCLE_TIME_MS - (OSTimeGet() - start_execution);

				if(timeToWait > 0)
					OSTimeDlyHMSM(0, 0, 0, timeToWait);
			}
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
               
  OSTaskCreateExt(speedControl,
                  NULL,
                  &speedControl_stk[TASK_STACKSIZE-1],
                  TASK_SPD_CTRL_PRIORITY,
                  TASK_SPD_CTRL_PRIORITY,
                  speedControl_stk,
                  TASK_STACKSIZE,
                  NULL,
                  0);

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
#endif

#ifdef TEST

#include <stdio.h>
#include <io.h>
#include "head.h"
#include "head.c"


//#define MOTOR_TEST
//#define HC_SR04_TEST
#define MPU_TEST
//#define COMPASS_TEST

typedef enum
{
	Ready,
	SearchParkingSlot,
	AntiClockRightAxis1,
	BackwardApproach1,
	AntiClock1,
	AntiClock2,
	BackwardApproach2,
	parked
} ParkingStateType_t;

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
