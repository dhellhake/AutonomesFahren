/*
 * Ultrasound.c
 *
 */

#include "../VMC.h"

/*
 * Initialisation for Ultrasound sensors with 0 values
 * (necessary before making measurements)
 */
void initUltrasoundSensors() {
	int i = 0;
	int j = 0;
	for (i = 0; i < NUMBER_OF_ULTRA_SOUND_DEVICES; i++) {
		values[i].counter = 0;
		for (j = 0; j < NUMBER_MEAN_VALUES; j++) {
			values[i].ultraSoundValues[j] = 0;
		}
	}
}

/*
 * Set sensor mask to make a ultrasound message from all sensors
 */
void makeMeasurement() {
	*pHc_sr04 = 0xff;
}

/*
 * get calculated mean sensor distance values (simple mean value filter)
 * @param unsigned int *means
 * 	pointer to store the calculated mean values into
 */
char getMeanSensorDistance(unsigned int *means) {
	unsigned int mean = 0;
	unsigned int sensor = 0;
	unsigned int x;
	unsigned int oldDist = 0;
	unsigned int absDist = 0;

	int i = 0;
	if (*pHc_sr04 != 0xff) {
		makeMeasurement();
		return -1;
	}
	for (sensor = 0; sensor < NUMBER_OF_ULTRA_SOUND_DEVICES; sensor++) {
		x = MeasureDistance(sensor);
		if (values[sensor].counter == 0) {
			oldDist = values[sensor].ultraSoundValues[3];
		} else {
			oldDist = values[sensor].ultraSoundValues[values[sensor].counter-1];
		}
		//Filter out not plausible values
		absDist = oldDist>x ? oldDist - x : x - oldDist;
		//printf("%u = %u\n", sensor, absDist);
		if (oldDist!=0 && absDist > ERROR_TOLERANCE) {
			x = oldDist;
		}
		values[sensor].ultraSoundValues[values[sensor].counter] = x;
		//printf("%u = %u\n", sensor, x);
		if (values[sensor].counter >= 3) {
			values[sensor].counter = 0;
		} else {
			values[sensor].counter++;
		}
		//printf("Sensor counter: %d\n", values[sensor].counter);
		mean = 0;
		for (i = 0; i < NUMBER_MEAN_VALUES; i++) {
			mean += values[sensor].ultraSoundValues[i];
		}
		means[sensor] = mean >> 2;
	}

	makeMeasurement();
	return 0;
}
