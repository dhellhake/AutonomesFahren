#ifndef _VMC_H_
#define _VMC_H_

#include <stdio.h>
#include <io.h>
#include "head.h"
#include "includes.h"

#define WHL_CIRCUMFERENCE 		200 // [mm]
#define WHL_TICKS_PER_REVOLUTION 	20 // [whl_tick]
#define DISTANCE_PER_WHL_TICK (WHL_CIRCUMFERENCE/WHL_TICKS_PER_REVOLUTION) // [mm/ whl_tick]

#define OS_TICKS_PER_USEC (OS_TICKS_PER_SEC/1000000)
#define OS_TICKS_PER_MSEC (OS_TICKS_PER_SEC/1000)

#define SPD_CTRL_CYCLE_TIME_MS 100

INT32S desired_speed = 500;
INT16S Kp_SpeedCtrl_num = 1;
INT16S Kp_SpeedCtrl_den = 1;
INT16S Ki_SpeedCtrl_num = 1;
INT16S Ki_SpeedCtrl_den = 1;
INT16S Kd_SpeedCtrl_num = 1;
INT16S Kd_SpeedCtrl_den = 1;
INT32S I_SpeedCtrl_min = -10000;
INT32S I_SpeedCtrl_max = 10000;
INT32S P_SpeedCtrl = 0;
INT32S I_SpeedCtrl = 0;
INT32S D_SpeedCtrl = 0;
INT16S PWM_SpeedCtrl = 0;
INT16S Fast_Forward_Control = 0;
INT32S I_SpeedCtrl_error = 0;
INT32U speed = 0;
INT32S e_speed = 0;
INT16S PWM_SpeedCtrl_max = 100;
INT16S PWM_SpeedCtrl_min = 0;

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

/*typedef enum
{
	true,
	false
} bool_t;*/

#endif
