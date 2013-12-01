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
#include "AttitudeLoop.h"

class AccGyro: public MPU6050DMP {
public:
	AccGyro(int devAddr);
	virtual ~AccGyro();

	void init();
	void exportValueToSerial();
	void exportTeaPot();

	bool checkDataAvailable();

	bool fillValues();

	bool readFromSerial();
	bool readFromSensor();

	void setSimulate(bool simulate) {Serial.print(F("Setting simulation mode:")); Serial.println(simulate); fSimulate = simulate;}

	void calibrate();
	void calibrateSerial();
	void calibrateSensor();

private:
	bool initialized;
	bool dmpInitialized;
	int address;
	AccGyroData data;

	uint8_t mpuIntStatus;
	uint16_t dmpPacketSize;

	uint16_t fifoCount;     // count of all bytes currently in FIFO

	bool fSimulate;
	int currentIndex;
	float buffer[10];

	AttitudeLoop quatPPLoop;
};

#endif /* ACCGYRO_H_ */
