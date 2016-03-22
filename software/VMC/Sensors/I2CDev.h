#ifndef I2CDEV_H_
#define I2CDEV_H_

#ifdef __cplusplus
class I2Cdev {
public:
	I2Cdev();

	static void readBit(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitNum, unsigned char *data);
	static void readBitW(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitNum, unsigned short *data);
	static void readBits(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitStart, unsigned char length, unsigned char *data);
	static void readBitsW(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitStart, unsigned char length, unsigned short *data);
	static void readByte(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char *data);
	static void readWord(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned short *data);
	static void readBytes(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char length, unsigned char *data);
	static void readWords(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char length, unsigned short *data);

	static void writeBit(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitNum, unsigned char data);
	static void writeBitW(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitNum, unsigned short data);
	static void writeBits(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitStart, unsigned char length, unsigned char data);
	static void writeBitsW(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char bitStart, unsigned char length, unsigned short data);
	static void writeByte(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char data);
	static void writeWord(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned short data);
	static void writeBytes(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char length, unsigned char *data);
	static void writeWords(unsigned char I2CAddr, unsigned char RegAddr,
			unsigned char length, unsigned short *data);
	static void readWriteWait();
};
#endif
#endif /* I2CDEV_H_ */
