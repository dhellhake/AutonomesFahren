#include "vmc.h"

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

			printf("g_i16s_PWMSpeedCtrl: %d\n", g_i16s_PWMSpeedCtrl);

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
