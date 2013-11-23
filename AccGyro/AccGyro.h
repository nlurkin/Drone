/*
 * AccGyro.h
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#ifndef ACCGYRO_H_
#define ACCGYRO_H_

#include <Wire.h>
#include <HardwareSerial.h>
#include "AccGyroData.h"
//#include "MPU6050.h"
#include "MPU6050DMP.h"

class AccGyro: public MPU6050DMP {
public:
	AccGyro(int devAddr);
	virtual ~AccGyro();

	void init();
	void exportValueToSerial();
	void exportTeaPot();

	bool checkDataAvailable();

	void fillValues();
private:
	bool initialized;
	bool dmpInitialized;
	int address;
	AccGyroData data;

	uint8_t mpuIntStatus;
	uint16_t dmpPacketSize;

	uint16_t fifoCount;     // count of all bytes currently in FIFO
	//uint8_t fifoBuffer[64]; // FIFO storage buffer

	/*Quaternion q;
	float euler[3];
	VectorFloat gravity;
	float ypr[3];
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;*/
};

#endif /* ACCGYRO_H_ */
