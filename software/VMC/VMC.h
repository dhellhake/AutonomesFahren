/** @file
 * Header file which contains all necessary declarations for the Vehicle Management and Control Project
 * All necessary header files are included in this file, thus it is only necessary to include this header file
 * in other source files.
 */

#ifndef _VMC_H_
#define _VMC_H_

/* --------------- Includes --------------- */
// Standard libraries
#include <stdio.h>
#include <io.h>
#include "math.h"
#include "string.h"
#include <iostream>
#include <cstdlib>

// uC-OSII/NIOS/Altera
#include "head.h"
#include "includes.h"
#include "time.h"
#include "altera_up_avalon_rs232.h"
#include "system.h"

// VMClib
#include "VMClib\VMClib.h"

// Sensors
#include "Sensors\Ultrasound.h"
#include "Sensors\helper_3dmathc.h"
#include "Sensors\mpu6050.h"

// MotionControl
#include "MotionControl\MotionControl.h"

/* --------------- Macros --------------- */
#define WHL_CIRCUMFERENCE 		200 // [mm]
#define WHL_TICKS_PER_REVOLUTION 	20 // [whl_tick]
#define DISTANCE_PER_WHL_TICK (WHL_CIRCUMFERENCE/WHL_TICKS_PER_REVOLUTION) // [mm/ whl_tick]

#define OS_TICKS_PER_USEC (OS_TICKS_PER_SEC/1000000)
#define OS_TICKS_PER_MSEC (OS_TICKS_PER_SEC/1000)

#define SPD_CTRL_CYCLE_TIME_MS 			100
#define SENSOR_COLLECTOR_CYCLE_TIME_MS 	60
#define EMERCENCY_STOP_CYCLE_TIME_MS	10

#define EMERGENCY_STOP_DISTANCE 100

/* --------------- Globals --------------- */
extern INT32S desired_speed;
extern INT16S Kp_SpeedCtrl_num;
extern INT16S Kp_SpeedCtrl_den;
extern INT16S Ki_SpeedCtrl_num;
extern INT16S Ki_SpeedCtrl_den;
extern INT16S Kd_SpeedCtrl_num;
extern INT16S Kd_SpeedCtrl_den;
extern INT32S I_SpeedCtrl_min;//-10000;
extern INT32S I_SpeedCtrl_max;//10000;
extern INT32S P_SpeedCtrl;
extern INT32S I_SpeedCtrl;
extern INT32S D_SpeedCtrl;
extern INT16S g_i16s_PWMSpeedCtrl;
extern INT16S Fast_Forward_Control;
extern INT32S I_SpeedCtrl_error;
extern INT32U speed;
extern INT32S e_speed;
extern INT32S e_speed_old;
extern INT16S PWM_SpeedCtrl_max;
extern INT16S PWM_SpeedCtrl_min;
extern INT32S step_size;
extern OS_EVENT *mutex;
extern INT8U return_code;

#endif
