/* steering.cpp */
//#include "vmc.h"

/*
INT16S calcSteeringOffset(INT16S steeringValue)
{
	INT16S pwmSteeringOffset = steeringValue >> 1;


	if(pwmSteeringOffset > 0)	// steer right
	{
		if(g_i16s_PWMSpeedCtrl > 50)
		{
			set_duty_cycle(pRearRightDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
			set_duty_cycle(pFrontRightDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
		}
		else
		{
			set_duty_cycle(pRearLeftDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
			set_duty_cycle(pFrontLeftDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
		}
	}
	else	// steer left
	{
		if(g_i16s_PWMSpeedCtrl > 50)
		{
			set_duty_cycle(pRearLeftDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
			set_duty_cycle(pFrontLeftDutySet, g_i16s_PWMSpeedCtrl-pwmSteeringOffset);
		}
		else
		{
			set_duty_cycle(pRearRightDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
			set_duty_cycle(pFrontRightDutySet, g_i16s_PWMSpeedCtrl+pwmSteeringOffset);
		}
	}

	return pwmSteeringOffset;
}*/
