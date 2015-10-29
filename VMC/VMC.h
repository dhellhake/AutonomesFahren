#ifndef _VMC_H_
#define _VMC_H_

#include <stdio.h>
#include <io.h>
#include "head.h"
#include "includes.h"

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
