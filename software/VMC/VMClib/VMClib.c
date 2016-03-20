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
	initUltrasoundSensors();

	pEmergencyStop = (volatile unsigned int*)(STATE_CMD_MEMORY_BASE | 0x04);
}

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

/* Task for Speed Control with PID-Regulator */
void speedControl(void *pdata)
{
	int time = 0;
	int time_old = 0;
	int delta_t = 0;

	/* local variables to store the individual wheel ticks */
	INT32U whl_ticks_fl = 0;
	INT32U whl_ticks_fr = 0;
	INT32U whl_ticks_rl = 0;
	INT32U whl_ticks_rr = 0;

	/* local variables to store the wheel ticks of the previous time step */
	INT32U whl_ticks_fl_old = 0;
	INT32U whl_ticks_fr_old = 0;
	INT32U whl_ticks_rl_old = 0;
	INT32U whl_ticks_rr_old = 0;

	/* local variables to store the change of the wheel ticks between the cycles */
	INT32U delta_whl_ticks_fl = 0;
	INT32U delta_whl_ticks_fr = 0;
	INT32U delta_whl_ticks_rl = 0;
	INT32U delta_whl_ticks_rr = 0;

	/* local variables to store the completed distance between the cycles in [mm]*/
	INT32U delta_s_fl = 0;
	INT32U delta_s_fr = 0;
	INT32U delta_s_rl = 0;
	INT32U delta_s_rr = 0;

	/* local variables to store the actual wheel speed in the current cycle */
	INT32U actual_speed_fl = 0;
	INT32U actual_speed_fr = 0;
	INT32U actual_speed_rl = 0;
	INT32U actual_speed_rr = 0;

	/* persistent arrays to store the speed values of the last four cycles
	 * to calculate the moving on mean
	 */
	static INT32U speed_fl[4] = {0, 0, 0, 0};
	static INT32U speed_fr[4] = {0, 0, 0, 0};
	static INT32U speed_rl[4] = {0, 0, 0, 0};
	static INT32U speed_rr[4] = {0, 0, 0, 0};

	/* index variable for calculating moving on mean */
	INT32U i = 0;

	/* local variables to measure execution time and to
	 * determine the wait time
	 */
	INT32U start_execution = 0;
	INT32U timeToWait = 0;

	while(1)
	{
		/* save starting time of each cycle */
		start_execution = OSTimeGet();

		/* save end time of last cycle for calculation of cycle time */
		time_old = time;
		/* Save actual time in [ms] */
		time = OSTimeGet() / OS_TICKS_PER_MSEC;
		/* calculate cycle time of previous cycle */
		delta_t = time - time_old;

		/* save number of wheel ticks of previous cycle */
		whl_ticks_fl_old = whl_ticks_fl;
		whl_ticks_fr_old = whl_ticks_fr;
		whl_ticks_rl_old = whl_ticks_rl;
		whl_ticks_rr_old = whl_ticks_rr;

		/* compensate wheel tick error at beginning by setting the
		 * wheel ticks of all four wheels to 0 for the first 200 ms
		 */
		if(time < 200)
		{
			whl_ticks_fl = 0;
			whl_ticks_fr = 0;
			whl_ticks_rl = 0;
			whl_ticks_rr = 0;
		}
		else
		{
			/* read the actual wheel ticks */
			whl_ticks_fl = *pFrontLeftEncRead;
			whl_ticks_fr = *pFrontRightEncRead;
			whl_ticks_rl = *pRearLeftEncRead;
			whl_ticks_rr = *pRearRightEncRead;
		}

		/* calculate number of wheel ticks in current cycle */
		delta_whl_ticks_fl = whl_ticks_fl - whl_ticks_fl_old;
		delta_whl_ticks_fr = whl_ticks_fr - whl_ticks_fr_old;
		delta_whl_ticks_rl = whl_ticks_rl - whl_ticks_rl_old;
		delta_whl_ticks_rr = whl_ticks_rr - whl_ticks_rr_old;

		/* conversion from [wheel_tick] to [mm] */
		delta_s_fl = delta_whl_ticks_fl * DISTANCE_PER_WHL_TICK;
		delta_s_fr = delta_whl_ticks_fr * DISTANCE_PER_WHL_TICK;
		delta_s_rl = delta_whl_ticks_rl * DISTANCE_PER_WHL_TICK;
		delta_s_rr = delta_whl_ticks_rr * DISTANCE_PER_WHL_TICK;

		/* adjust index counter for moving on mean calculation */
		if(i > 3)
		{
			i = 0;
		}

		/* calculate speed of the individual wheels in [mm/s] */
		speed_fl[i] = delta_s_fl*1000/delta_t;
		speed_fr[i] = delta_s_fr*1000/delta_t;
		speed_rl[i] = delta_s_rl*1000/delta_t;
		speed_rr[i] = delta_s_rr*1000/delta_t;

		/* ATTENTION: FIR-Filter leads to attack-time of regulator
		 * calculation of actual speed for each wheel by using
		 * moving on mean filter. Using 4 values for calculation
		 * makes it possible to use bit shifting and therefore
		 * leads to an efficient implementation.
		 */
		actual_speed_fl = (speed_fl[0] + speed_fl[1] + speed_fl[2] + speed_fl[3]) >> 2;
		actual_speed_fr = (speed_fr[0] + speed_fr[1] + speed_fr[2] + speed_fr[3]) >> 2;
		actual_speed_rl = (speed_rl[0] + speed_rl[1] + speed_rl[2] + speed_rl[3]) >> 2;
		actual_speed_rr = (speed_rr[0] + speed_rr[1] + speed_rr[2] + speed_rr[3]) >> 2;

		/* calculating overall speed of car.
		 * since the wheel sensors are not very accurate and
		 * have some undeterministic disturbances especially on
		 * the front axle, only the values of the rear wheels
		 * are used to calculate the overall speed of the car
		 * in this version.
		 */
		speed = ((actual_speed_rl + actual_speed_rr) >> 1);

		/* calculation of the control error e */
		e_speed = desired_speed - speed;

		/* calculation of the P-component of the PID-Controller */
		P_SpeedCtrl = ((Kp_SpeedCtrl_num * e_speed) / Kp_SpeedCtrl_den);

		/* calculation of the I-component of the PID-Controller */
		I_SpeedCtrl_error = I_SpeedCtrl_error + e_speed;

		/* limitation of the I-Component */
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

		/* calculation of the fast-forward control */
		Fast_Forward_Control = ((36 * desired_speed)/1000) + 18;

		/* calculating corresponding duty-cycle for PWM-Signal for the motors */
		g_i16s_PWMSpeedCtrl = (INT16S) ((P_SpeedCtrl + I_SpeedCtrl + D_SpeedCtrl) + Fast_Forward_Control);

		/* limitation of actuator signal */
		if(g_i16s_PWMSpeedCtrl < PWM_SpeedCtrl_min)
		{
			g_i16s_PWMSpeedCtrl = PWM_SpeedCtrl_min;
		}
		else if(g_i16s_PWMSpeedCtrl > PWM_SpeedCtrl_max)
		{
			g_i16s_PWMSpeedCtrl = PWM_SpeedCtrl_max;
		}

		/* set the duty-cycle for the wheel motors */
		set_duty_cycle( pFrontRightDutySet, g_i16s_PWMSpeedCtrl);
		set_duty_cycle(pRearRightDutySet, g_i16s_PWMSpeedCtrl);
		set_duty_cycle(pRearLeftDutySet, g_i16s_PWMSpeedCtrl);
		set_duty_cycle(pFrontLeftDutySet, g_i16s_PWMSpeedCtrl);

		/* enable the motors for driving all wheels forward */
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
		printf("g_i16s_PWMSpeedCtrl: %d\n", g_i16s_PWMSpeedCtrl);

		printf("actual_speed_fl: %d\n", actual_speed_fl);
		printf("actual_speed_fr: %d\n", actual_speed_fr);
		printf("actual_speed_rl: %d\n", actual_speed_rl);
		printf("actual_speed_rr: %d\n", actual_speed_rr);

		printf("speed: %d\n", speed);
		printf("actual_speed_rr: %d\n", actual_speed_rr);
		printf("desired_speed: %d\n", desired_speed);
#endif
		/* increment index variable for moving on mean calculation */
		i = i + 1;

		if(*pEmergencyStop == 1)
		{
			printf("EmergencyStop: %d\n", *pEmergencyStop);
			set_duty_cycle( pFrontRightDutySet, 0);
			set_duty_cycle(pRearRightDutySet, 0);
			set_duty_cycle(pRearLeftDutySet, 0);
			set_duty_cycle(pFrontLeftDutySet, 0);
		}

		timeToWait = SPD_CTRL_CYCLE_TIME_MS - (OSTimeGet() - start_execution);

		/* if actual execution time is less than intended
		 * cycle time, then wait.
		 */
		if(timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}

INT16S calcSteeringOffset(INT16S steeringValue)
{
	INT16S pwmSteeringOffset = steeringValue / 2;

	set_duty_cycle(pRearRightDutySet, g_i16s_PWMSpeedCtrl);
	set_duty_cycle(pFrontRightDutySet, g_i16s_PWMSpeedCtrl);
	set_duty_cycle(pRearLeftDutySet, g_i16s_PWMSpeedCtrl);
	set_duty_cycle(pFrontLeftDutySet, g_i16s_PWMSpeedCtrl);

	if(pwmSteeringOffset > 0)	// steer right
	{
		if(g_i16s_PWMSpeedCtrl > 50)
		{
			set_duty_cycle(pRearRightDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
			set_duty_cycle(pFrontRightDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
#ifdef DEBUG
			printf("g_i16s_PWMSpeedCtrl-pwmSteeringOffset: %d\n", g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
#endif
		}
		else
		{
			set_duty_cycle(pRearLeftDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
			set_duty_cycle(pFrontLeftDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
#ifdef DEBUG
			printf("g_i16s_PWMSpeedCtrl+pwmSteeringOffset: %d\n", g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
#endif
		}
	}
	else	// steer left
	{
		if(g_i16s_PWMSpeedCtrl > 50)
		{
			set_duty_cycle(pRearLeftDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
			set_duty_cycle(pFrontLeftDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
#ifdef DEBUG
			printf("g_i16s_PWMSpeedCtrl+pwmSteeringOffset: %d\n", g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
#endif
		}
		else
		{
			set_duty_cycle(pRearRightDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
			set_duty_cycle(pFrontRightDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
#ifdef DEBUG
			printf("g_i16s_PWMSpeedCtrl-pwmSteeringOffset: %d\n", g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
#endif
		}
	}

#ifdef DEBUG
	printf("pwmSteeringOffset: %d\n", pwmSteeringOffset);
#endif
	return pwmSteeringOffset;
}

void initUltrasoundSensors()
{
	int i = 0;
	int j = 0;
	for (i = 0; i < NUMBER_OF_ULTRA_SOUND_DEVICES; i++) {
		values[i].counter = 0;
		for (j = 0; j < NUMBER_MEAN_VALUES; j++) {
			values[i].ultraSoundValues[j] = 0;
		}
	}
}

void startMeasurement(INT8U sensorIndex)
{
	*pHc_sr04 |= sensorIndex;
}

void makeMeasurement()
{
	*pHc_sr04 = 0xff;
}

char getMeanSensorDistance(unsigned int *means, INT8U sensorIndex)
{
	unsigned int mean = 0;
	unsigned int sensor = 0;
	unsigned int x;
	unsigned int oldDist = 0;
	unsigned int absDist = 0;

	int i = 0;
	if (*pHc_sr04 != 0xff)
	{
		startMeasurement(sensorIndex);

		return -1;
	}

	for (sensor = 0; sensor < NUMBER_OF_ULTRA_SOUND_DEVICES; sensor++)
	{
		x = MeasureDistance(sensor);

		if (values[sensor].counter == 0)
		{
			oldDist = values[sensor].ultraSoundValues[3];
		}
		else
		{
			oldDist = values[sensor].ultraSoundValues[values[sensor].counter-1];
		}

		//Filter out not plausible values
		absDist = oldDist>x ? oldDist - x : x - oldDist;
		//printf("%u = %u\n", sensor, absDist);

		if (oldDist!=0 && absDist > ERROR_TOLERANCE)
		{
			x = oldDist;
		}

		values[sensor].ultraSoundValues[values[sensor].counter] = x;
		//printf("%u = %u\n", sensor, x);

		if (values[sensor].counter >= 3)
		{
			values[sensor].counter = 0;
		}
		else
		{
			values[sensor].counter++;
		}
		//printf("Sensor counter: %d\n", values[sensor].counter);
		mean = 0;
		for (i = 0; i < NUMBER_MEAN_VALUES; i++)
		{
			mean += values[sensor].ultraSoundValues[i];
		}
		means[sensor] = mean >> 2;
	}

	startMeasurement(sensorIndex);
	return 0;
}
