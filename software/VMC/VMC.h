#ifndef _VMC_H_
#define _VMC_H_

#include <stdio.h>
#include <io.h>
#include "head.h"
#include "includes.h"

#define WHL_CIRCUMFERENCE 		205 // [mm]
#define TICKS_PER_REVOLUTION 	21
#define OS_TICKS_PER_USEC (OS_TICKS_PER_SEC/1000000)

int desired_speed = 0;

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

typedef enum
{
	true,
	false
} bool_t;

#endif
