/*
 * VCMlib.h
 *
 *  Created on: 07.01.2016
 *      Author: Dominik Hellhake (3662000)
 */

#ifndef VCMLIB_H_
#define VCMLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* --------------- Globals --------------- */
INT8U *pEmergencyStop;

/* --------------- Macros --------------- */
#define SNR_DMP_BASE (0x80000000 | STATE_CMD_MEMORY_BASE)
#define SNR_SONIC_BASE ((0x80000000 | STATE_CMD_MEMORY_BASE) + sizeof(snr_dmp_t))
#define ACT_WHL_BASE ((0x80000000 | STATE_CMD_MEMORY_BASE) + sizeof(snr_dmp_t) + (sizeof(snr_sonic_t) * 8))
#define EMCY_STP_BASE ((0x80000000 | STATE_CMD_MEMORY_BASE) + sizeof(snr_dmp_t) + (sizeof(snr_sonic_t) * 8) + sizeof(act_wheel_t) * 4)
#define NUMBER_MEAN_VALUES 4
#define ERROR_TOLERANCE 1000
#define MUTEX_PRIORITY 5

/* --------------- Datatypes --------------- */

/*
 * At MANUER_QUEUE_BASE address a struct of mnv_queue_t
 * is stored.
 * From (MNV_QUEUE_BASE + sizeof(mnv_queue_t)) to (MNV_QUEUE_BASE + MANUER_QUEUE_SPAN) bytes
 * are used to store mnv_item_t items.
 */
#define MNV_QUEUE_BASE (0x80000000 | MANUER_QUEUE_BASE)
/*
 * Structure representing a maneuver item
 * property: _type defines the type of maneuver
 * property: _dequeued flag indicating if the maneuver has been executed
 */
typedef struct
{
	unsigned int _type;
	volatile unsigned int _dequeued;
} mnv_item_t;
/*
 * Structure representing a maneuver queue
 * property: _read position of the manuever_item to be read next
 * property: _write target position of the next write
 * property: _buffer pointer to the first manuever_item in memory (base-address)
 */
typedef struct
{
	unsigned int _read;
	unsigned int _write;
	volatile mnv_item_t* _buffer;
} mnv_queue_t;

enum SONIC_SENSOR_POS { VL = 7, VM = 0, VR = 1, HL = 5, HM = 4, HR = 3, SL = 6, SR = 2 };
/*
 * Structure representing a measured distance of a ultrasonic sensor
 * property: _distance the distance measured in centimeter
 * property: _timestamp the timestamp at which the measuremend has been made
 * property: _position the position of the ultrasonic sensor which has been used for measureing
 */
typedef struct
{
	unsigned int _distance;
	unsigned int _timestamp;
	enum SONIC_SENSOR_POS _position;
} snr_sonic_t;

enum WHEEL_POS { WHL_VL = 0, WHL_VR = 1, WHL_HL = 2, WHL_HR = 3 };
/*
 * Structure representing the state of a wheel
 * property: _distance the covered distance of the wheel since system-start or reset millimeter
 * property: _timestamp the timestamp at which the measuremend has been made
 * property: _position the position of the wheel
 * property: _ticks of the wheel-sensor
 */
typedef struct
{
	int _ticks;
	unsigned int _distance;
	unsigned int _timestamp;
	enum WHEEL_POS _position;
} act_wheel_t;

/*
 * Structure representing a valueset measured by the dmp
 * property: _yaw of the vehicle since system-start or reset measured in degree
 * property: _pitch pitch of the vehicle since system-start or reset measured in degree
 * property: _roll roll of the vehicle since system-start or reset measured in degree
 * property: _accX acceleration along X axis measured in m/s²
 * property: _accY acceleration along Y axis measured in m/s²
 * property: _accZ acceleration along Z axis measured in m/s²
 */
typedef struct
{
	int _yaw;
	int _pitch;
	int _roll;
	int _accX;
	int _accY;
	int _accZ;
	unsigned int _timestamp;
} snr_dmp_t;

typedef struct SensorValue
{
	unsigned int ultraSoundValues[NUMBER_MEAN_VALUES];
	int counter;
}SENSOR_VALUE;

/* --------------- Prototypes --------------- */
extern snr_sonic_t* SONICGetState(enum SONIC_SENSOR_POS position);
extern void SONICSetState(unsigned int distance, enum SONIC_SENSOR_POS position);
extern void MNV_IntQueue();
extern inline mnv_item_t* MNV_Queue_DeQueue();
extern inline mnv_item_t* MNV_Queue_EnQueue(unsigned int type);
extern inline unsigned int MNV_Queue_Item_Available();
extern void MNV_Queue_Test(void *pdata);
extern snr_dmp_t* DMPGetValueSet();
extern void DMPSetValueSet(int yaw, int pitch, int roll, int accX, int accY, int accZ);
extern void speedControl(void *pdata);
extern INT16S calcSteeringOffset(INT16S steeringValue);
char getMeanSensorDistance(unsigned int *means);
void makeMeasurement();
void initUltrasoundSensors();
void initVMC(void);
SENSOR_VALUE values[NUMBER_OF_ULTRA_SOUND_DEVICES];
extern act_wheel_t* WHLGetState();
extern void WHLSetState(int ticks, int distance, enum WHEEL_POS position);

#ifdef __cplusplus
}
#endif

#endif /* VCMLIB_H_ */
