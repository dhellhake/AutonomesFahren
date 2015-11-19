#include <stdio.h>
#include <io.h>
#include "head.h"


//#define MOTOR_TEST
#define HC_SR04_TEST
//#define MPU_TEST
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

typedef enum
{
	true,
	false
} bool_t;

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
