/** @file
 *@brief I2CDev.cpp
*/

#include "I2CDev.h"

extern "C" {
#include "../head.h"
}
/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

void I2Cdev::readBit(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitNum, unsigned char *data) {
	char b;
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, 1, &b);
	*data = b & (1 << bitNum);
}

void I2Cdev::readBitW(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitNum, unsigned short *data) {
	short w;
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, 2, (char*) &w);
	*data = w & (1 << bitNum);
}

void I2Cdev::readBits(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitStart, unsigned char length, unsigned char *data) {
	char b;
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, 1, &b);
	char mask = ((1 << length) - 1) << (bitStart - length + 1);
	b &= mask;
	b >>= (bitStart - length + 1);
	*data = b;
}

void I2Cdev::readBitsW(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitStart, unsigned char length, unsigned short *data) {
	short w;
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, 2, (char*) &w);
	short mask = ((1 << length) - 1) << (bitStart - length + 1);
	w &= mask;
	w >>= (bitStart - length + 1);
	*data = w;
}

void I2Cdev::readByte(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char *data) {
	char b;
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, 1, &b);
	*data = b;
}

void I2Cdev::readWord(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned short *data) {
	char w[2];
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, 2, w);
	//*data = w[0];
	//*data = w[1];
}

void I2Cdev::readBytes(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char length, unsigned char *data) {
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, length, (char*) data);
}

void I2Cdev::readWords(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char length, unsigned short *data) {
	readWriteWait();
	I2CRead(I2CAddr, RegAddr, length * 2, (char*) data);
}

/*
 * void I2CWrite(unsigned char I2CAddr, unsigned char RegAddr, unsigned char data)
 {
 *pI2C = I2CAddr; // FIrst write which I2C module is being addressed. MPU or HMC
 *pI2CReg = RegAddr; // write the address of register to which you want to write
 *pI2CRegWriteData = data; // write the data
 *pI2CDataLen = 1; // currently only 1 is supported for write access
 *pI2CCmd = (I2C_WRITE_CMD_MASK | I2C_START_CMD_MASK); // give write+start command
 }
 */

void I2Cdev::writeBit(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitNum, unsigned char data) {
	unsigned char b;
	readByte(I2CAddr, RegAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr, b);
}

/*
 * Might not work due to write restrictions
 */
void I2Cdev::writeBitW(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitNum, unsigned short data) {
	unsigned short w;
	readWord(I2CAddr, RegAddr, &w);
	w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr, (char) w);
	w >>= 8;
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr + 1, (char) w);
}

void I2Cdev::writeBits(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitStart, unsigned char length, unsigned char data) {
	unsigned char b;
	readByte(I2CAddr, RegAddr, &b);
	char mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1);
	data &= mask;
	b &= ~(mask);
	b |= data;
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr, (char) b);
}

void I2Cdev::writeBitsW(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char bitStart, unsigned char length, unsigned short data) {

	unsigned short w;
	readWord(I2CAddr, RegAddr, &w);
	short mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1);
	data &= mask;
	w &= ~(mask);
	w |= data;
	writeWord(I2CAddr, RegAddr, w);

}

void I2Cdev::writeByte(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char data) {
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr, (char) data);
}

void I2Cdev::writeWord(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned short data) {
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr, (char) data);
	data >>= 8;
	readWriteWait();
	I2CWrite(I2CAddr, RegAddr + 1, (char) data);
}

void I2Cdev::writeBytes(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char length, unsigned char *data) {
	int i;
	for (i = 0; i < length; i++) {
		readWriteWait();
		char send = data[i];
		I2CWrite(I2CAddr, RegAddr, (char) data[i]);
	}

}

void I2Cdev::writeWords(unsigned char I2CAddr, unsigned char RegAddr,
		unsigned char length, unsigned short *data) {
	int i;
	readWriteWait();
	for (i = 0; i < length; i++) {
		I2Cdev::writeWord(I2CAddr, RegAddr + (i * 2), data[i]);
		/*
		I2CWrite(I2CAddr, RegAddr + i * 2, (char) data[i]);
		data[i] >>= 8;
		I2CWrite(I2CAddr, RegAddr + i * 2 + 1, (char) data[i]);
		*/
	}
}

void I2Cdev::readWriteWait() {
	int counter = 20000;
		while (counter > 0) {
			counter--;
		}
}

