/** @file
 *  VMC.c
 * @brief Main file of the Vehicle Management and Control application
 * of the Seminar-Course Autonomous Driving (IN4804) in WS15/16
 *
 * Team Members:
 * Dominik Hellhake (03662000)
 * Sebastian Pretschner
 * Maximilian Schmitt (03665464)
 *
 * Version: 1.0 (final version for grading)
 * Date: 2016-03-22
 *
 * TODO:
 *  - include mutex in getter/setter functions if possible (partially done for wheel ticks)
 *  - refactore code and implement coding guidelines to increase quality and maintainability
 */

/**
 * \def Debug-Switch to disable debug printouts.
 * Note that debug printouts produce a large overhead and have very big influence on
 * timing and stability of the complete application and thus should be used with care.
 */
//#define DEBUG
//#define TEST

#include "VMC.h"

/*
 * \def Since the original library of the MPU6050 was written in C++ we
 * kept the capability to compile it either as C or C++ application. Therefore
 * some mechanisms have to be implemented to run C functions in C++ and vice versa.
 */
#ifdef __cplusplus
using namespace std;
#endif

// Initialization of global variables
INT32S desired_speed = 500;		// [mm/s]
INT16S Kp_SpeedCtrl_num = 296;
INT16S Kp_SpeedCtrl_den = 1000;
INT16S Ki_SpeedCtrl_num = 897;
INT16S Ki_SpeedCtrl_den = 1000;
INT16S Kd_SpeedCtrl_num = 0;
INT16S Kd_SpeedCtrl_den = 1000;
INT32S I_SpeedCtrl_min = -100; //-10000;
INT32S I_SpeedCtrl_max = 100; //10000;
INT32S P_SpeedCtrl = 0;
INT32S I_SpeedCtrl = 0;
INT32S D_SpeedCtrl = 0;
INT16S g_i16s_PWMSpeedCtrl = 0;		// [%]
INT16S Fast_Forward_Control = 0;
INT32S I_SpeedCtrl_error = 0;
INT32U speed = 0;
INT32S e_speed = 0;
INT32S e_speed_old = 0;
INT16S PWM_SpeedCtrl_max = 80; // more than 80% PWM-DutyCycle leads to faulty values of the wheel encoders
INT16S PWM_SpeedCtrl_min = 0;
INT32S step_size = 50;
OS_EVENT *mutex = NULL;
INT8U return_code = OS_NO_ERR;

#ifndef TEST

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK sensorCollector_stk[TASK_STACKSIZE];
OS_STK speedControl_stk[TASK_STACKSIZE];
OS_STK emcyStp_stk[TASK_STACKSIZE];
OS_STK test_stk[TASK_STACKSIZE];

/**\def Definition of Task Priorities */
/* Each Task must have a unique priority number
 * between 0 and 254. Priority level 0 is the
 * highest priority.
 */
#define TASK_EMCY_STP_PRIORITY      	1
#define TASK_SENSOR_COLLECTOR_PRIORITY  2
#define TASK_SPD_CTRL_PRIORITY			3
#define TASK_TEST_PRIORITY				4

/** \fn void SensorCollector(void* pdata)
 *  \brief This Task collects the the data of all equipped sensors of the car cyclic.
 *  this contains the MPU6050 provides the values for acceleration and angles of rotation
 *  in all degrees of freedom, the Ultrasound-Devices for the object and environment perception
 *  as well as the wheel encoders.
 *
 *  The cycle time of the task is 60 ms which is the recommended cycle time in the data sheet
 *  of the Ultrasound-Devices. Note that the task is very sensitive in relation to the cycle time
 *  and changes in the task cycle times likely lead to errors in the data evaluation and calculation
 *  and make the task very prone to crashes.
 *
 *  The task utilizes the getter and setter functions to access the state and command memory which is
 *  defined in the system.h. Note that every read/write access to the state and command memory needs to
 *  be protected by a mutex or else the application is likely to crash. For a better usability and to make
 *  the usage of the getter/setter functions less error prone the protection should be integrated in the
 *  functions itself in future versions.
 */

void sensorCollector(void* pdata)
{
	INT32U start_execution;
	INT32U timeToWait;

	// Ultrasound
	unsigned int ultraSoundSensors[8];
	int sensorCounter = 0;

	// Emergency Stop
	char emergencyStop = 0;

	// DMP
	unsigned short dmpPacketSize = 0;
	unsigned short fifoCount = 0;
	unsigned char fifoBuffer[64];
	unsigned char fifoBufferTmp[64];
	unsigned char mpuStatus = 0;
	CQuaternion q;
	CVectorFloat gravity;
	CVectorInt16 accl;
	float yawPitchRol[3];

	/* ------------ Start reading Ultrasound Devices ------------*/
	dmpPacketSize = mpuDmpGetFIFOPacketSize(myMPU);
	mpuResetFIFO(myMPU);
	mpuSetDMPEnabled(myMPU, 1);

	while (1)
	{
		start_execution = OSTimeGet();

		if (getMeanSensorDistance(ultraSoundSensors) == 0)
		{
			for (sensorCounter = 0;
					sensorCounter < NUMBER_OF_ULTRA_SOUND_DEVICES;
					sensorCounter++)
			{
#ifdef DEBUG
				printf("Sensor %i: %i [mm]\n", sensorCounter,
						ultraSoundSensors[sensorCounter]);
#endif

				OSMutexPend(mutex, 0, &return_code);
				SONICSetState(ultraSoundSensors[sensorCounter],
						(enum SONIC_SENSOR_POS) sensorCounter);
				OSMutexPost(mutex);

				if (ultraSoundSensors[sensorCounter] <= EMERGENCY_STOP_DISTANCE)
				{
					//TODO: We should raise a stop signal here
#ifdef DEBUG
					printf("Emergency stop!\n");
#endif
					emergencyStop = 1;
				}
			}
			OSMutexPend(mutex, 0, &return_code);
			*pEmergencyStop = emergencyStop;
			OSMutexPost(mutex);
			emergencyStop = 0;
		}

		/* ------------ Start reading DMP/MPU ------------*/
		fifoCount = mpuGetFIFOCount(myMPU);
		mpuStatus = mpuGetIntStatus(myMPU);

		if ((mpuStatus & 0x10) || fifoCount == 1024)
		{
			mpuResetFIFO(myMPU);
			printf("FIFO Overflow, reset\n");
		}
		else if (mpuStatus & 0x02)
		{
			//Read all old values and wait for newest to be written
			while ((fifoCount - dmpPacketSize) >= dmpPacketSize)
			{
				//Read till we have the last packet in the fifo, to avoid overflows
				mpuGetFIFOBytes(myMPU, fifoBuffer, dmpPacketSize);
				fifoCount = fifoCount - dmpPacketSize;
				//fifoCount = mpuGetFIFOCount(myMPU);
				//printf("Read to avoid overflow, paketsize is: %d, fifocount is: %d\n", dmpPacketSize, fifoCount);
			}

			while (fifoCount < dmpPacketSize)
			{
				fifoCount = mpuGetFIFOCount(myMPU);
			}
			mpuGetFIFOBytes(myMPU, fifoBuffer, dmpPacketSize);

			//gyro = mpuGetGyro(myMPU, fifoBuffer);
			memcpy(fifoBufferTmp, fifoBuffer, sizeof(char) * 64);
			mpuDmpGetQuaternion(myMPU, &q, fifoBufferTmp);
			mpuDmpGetGravity(myMPU, &gravity, &q);

			mpuDmpGetYawPitchRoll(myMPU, yawPitchRol, &q, &gravity);

			memcpy(fifoBufferTmp, fifoBuffer, sizeof(char) * 64);
			mpuDmpGetAccel(myMPU, &accl, fifoBufferTmp);
			//printf("Accl x: %d, y: %d, z: %d\n", accl.x, accl.y, accl.z);
			//printf("Gyro x: %d, y: %d, z: %d\n", gyro.x, gyro.y, gyro.z);
			printf("Yaw  %f, Pitch: %f, Roll: %f\n",
					yawPitchRol[2] * (180.0 / M_PI),
					yawPitchRol[1] * (180.0 / M_PI),
					yawPitchRol[0] * (180.0 / M_PI));

			OSMutexPend(mutex, 0, &return_code);
			DMPSetValueSet(yawPitchRol[2] * (180.0 / M_PI),
					yawPitchRol[1] * (180.0 / M_PI),
					yawPitchRol[0] * (180.0 / M_PI), accl.x, accl.y, accl.z);
			OSMutexPost(mutex);
		}


		/* ------------ Start reading Wheel Encoders ------------*/
		WHLSetState(*pFrontLeftEncRead, 0, WHL_VL);
		//WHLSetState(100, 0, WHL_VL);
		WHLSetState(*pFrontRightEncRead, 0, WHL_VR);
		WHLSetState(*pRearLeftEncRead, 0, WHL_HL);
		WHLSetState(*pRearRightEncRead, 0, WHL_HR);

		// Calculation of timeToWait for cycle time
		timeToWait = SENSOR_COLLECTOR_CYCLE_TIME_MS
				- (OSTimeGet() - start_execution);

		if (timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}

void emergencyStop(void* pdata)
{
	INT32U start_execution;
	INT32U timeToWait;

	INT8U emergencyStop = 0;

	while(1)
	{
		OSMutexPend(mutex, 0, &return_code);
		emergencyStop = *pEmergencyStop;
		OSMutexPost(mutex);

		if(emergencyStop == 1)
		{
			printf("EmergencyStop: %d\n", emergencyStop);
			set_duty_cycle( pFrontRightDutySet, 0);
			set_duty_cycle(pRearRightDutySet, 0);
			set_duty_cycle(pRearLeftDutySet, 0);
			set_duty_cycle(pFrontLeftDutySet, 0);
		}
		else
			; // no action necessary since Speed Controller sets PWM automatically back to intended value

		// Calculation of timeToWait for cycle time
		timeToWait = EMERCENCY_STOP_CYCLE_TIME_MS
				- (OSTimeGet() - start_execution);

		if (timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}

/** \fn void testSteeringInterface(void* pdata)
 *  \brief This is a test task to demonstrate the functionality of the
 *  implementet steering interface.
 */
void testSteeringInterface(void* pdata)
{
	int i = 0;
	g_i16s_PWMSpeedCtrl = 40;
	*pwm_enable = (ALL_WHEEL_FWD_MASK | ENABLE_ENC_MASK);

	calcSteeringOffset(33);
	OSTimeDlyHMSM(0, 0, 3, 0);

	calcSteeringOffset(-33);
	OSTimeDlyHMSM(0, 0, 3, 0);

	calcSteeringOffset(66);
	OSTimeDlyHMSM(0, 0, 3, 0);

	calcSteeringOffset(-66);
	OSTimeDlyHMSM(0, 0, 3, 0);

	calcSteeringOffset(100);
	OSTimeDlyHMSM(0, 0, 3, 0);

	calcSteeringOffset(-100);
	OSTimeDlyHMSM(0, 0, 3, 0);
}

/** \fn void readValues(void* pdata)
 *  \brief This is a test task to demonstrate the functionality of the
 *  getter and setter functions for the state and command memory. It utilizes
 *  the functions of VMClib to read out the values in the state and command
 *  memory that have been stored there previously by the sensorCollector Task.
 */
void readValues(void* pdata)
{
	INT32U start_execution;
	INT32U timeToWait;

	unsigned int ultraSoundSensors[8];
	int sensorCounter = 0;

	while (1)
	{
		snr_sonic_t* ultrasonic_test;
		snr_dmp_t* mpu_test;
		act_wheel_t* wheel_enc_test;

		start_execution = OSTimeGet();

		OSMutexPend(mutex, 0, &return_code);
		ultrasonic_test = SONICGetState(3);
		OSMutexPost(mutex);

		OSMutexPend(mutex, 0, &return_code);
		mpu_test = DMPGetValueSet();
		OSMutexPost(mutex);

		printf("Sensor 3: %i\n", ultrasonic_test->_distance);
		printf("Acc X: %i\n", mpu_test->_accX);

		wheel_enc_test =  WHLGetState(WHL_VL);

		printf("WheelTicks VL CMD/STATE MEM: %i WheelTicks VL direct: %i\n", wheel_enc_test->_ticks, *pFrontLeftEncRead);

		timeToWait = SENSOR_COLLECTOR_CYCLE_TIME_MS
				- (OSTimeGet() - start_execution);
		if (timeToWait > 0)
			OSTimeDlyHMSM(0, 0, 0, timeToWait);
	}
}

/** \fn int main(void)
 *  \brief This is the main task of the applications. It carries out all necessary initializations
 *  and then starts the previously defined tasks.
 */
int main(void) {
	INT8U task_status = OS_NO_ERR;

	OSInit();

	init();

	initVMC();

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

	/*OSTaskCreateExt(speedControl,
	 NULL,
	 &speedControl_stk[TASK_STACKSIZE-1],
	 TASK_SPD_CTRL_PRIORITY,
	 TASK_SPD_CTRL_PRIORITY,
	 speedControl_stk,
	 TASK_STACKSIZE,
	 NULL,
	 0);*/

	task_status = OSTaskCreateExt(sensorCollector,
			NULL,
			(void *) &sensorCollector_stk[TASK_STACKSIZE - 1],
			TASK_SENSOR_COLLECTOR_PRIORITY,
			TASK_SENSOR_COLLECTOR_PRIORITY,
			sensorCollector_stk,
			TASK_STACKSIZE,
			NULL,
			0);

#ifdef DEBUG
	printf("task status: %i; OS_NO_ERROR: %i\n", task_status, OS_NO_ERR);
#endif

	OSTaskCreateExt(readValues,
			NULL,
			&test_stk[TASK_STACKSIZE - 1],
			TASK_TEST_PRIORITY,
			TASK_TEST_PRIORITY,
			test_stk,
			TASK_STACKSIZE,
			NULL,
			0);

	/*OSTaskCreateExt(emergencyStop,
				NULL,
				&emcyStp_stk[TASK_STACKSIZE - 1],
				TASK_EMCY_STP_PRIORITY,
				TASK_EMCY_STP_PRIORITY,
				emcyStp_stk,
				TASK_STACKSIZE,
				NULL,
				0);*/

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
}bool_t;
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
		while (*pHc_sr04 != ULTRA_SOUND_READY_ALL_MASK);
		//while (*pHc_sr04 != (ULTRA_SOUND_0_READY_MASK | ULTRA_SOUND_1_READY_MASK));

		//printf("2\n");
		*pHc_sr04 = (ULTRA_SOUND_0_TRIG_CMD | ULTRA_SOUND_1_TRIG_CMD);
		//*pHc_sr04 = 0xC7;

		//printf("3\n");
		//while (*pHc_sr04 != (ULTRA_SOUND_0_READY_MASK | ULTRA_SOUND_1_READY_MASK));
		while (*pHc_sr04 != ULTRA_SOUND_READY_ALL_MASK);
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
