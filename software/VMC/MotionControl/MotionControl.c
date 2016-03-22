/** @file
 * MotionControl.c
 *
 * \brief Source file that contains all files necessary to control the motion of the car.
 */

#include "../VMC.h"

/** \fn void speedControl(void* pdata)
 *  \brief Task for Speed Control with PID-Regulator
 *
 */
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
		/*
		if(*pEmergencyStop == 1)
		{
			printf("EmergencyStop: %d\n", *pEmergencyStop);
			set_duty_cycle( pFrontRightDutySet, 0);
			set_duty_cycle(pRearRightDutySet, 0);
			set_duty_cycle(pRearLeftDutySet, 0);
			set_duty_cycle(pFrontLeftDutySet, 0);
		}*/

		timeToWait = SPD_CTRL_CYCLE_TIME_MS - (OSTimeGet() - start_execution);

		/* if actual execution time is less than intended
		 * cycle time, then wait.
		 */
		if(timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}

/** \fn INT16S calcSteeringOffset(INT16S steeringValue)
 *  \brief This task emulates a conventional steering of a car by
 *  calculating a PWM-Offset to the actual PWM of the drive motors
 *  to make the car move in the desired direction.
 *
 *  \param steeringValue Value that indicates the desired direction. Domain: [-100;100], where -100 corresponds to full left and 100 corresponds to full right
 *  \return The offset value for the motor control drive as PWM in [%]
 */
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


