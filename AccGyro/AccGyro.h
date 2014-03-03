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
#include "MPU6050DMP.h"
#include "SerialInterface.h"
#include "GenericSensor.h"

class AccGyro: public MPU6050DMP, public GenericSensor {
public:
	AccGyro(int devAddr);
	virtual ~AccGyro();

	void init();
	void exportValueToSerial();
	void exportTeaPot();
	Quaternion getQuaternion();
	VectorFloat getOmega();
	VectorFloat getAcceleration();
	VectorFloat getPosition();
	VectorFloat getAlpha();

	bool checkDataAvailable();
	bool fillValues();

	void setSimulate(bool simulate) {Serial.print(F("Setting simulation mode:")); Serial.println(simulate); fSimulate = simulate;}

	void calibrate();
	void setCalibration();

	void setSerialInterface(SerialInterface *s);

private:
	bool readFromSerial();
	bool readFromSensor();

	void calibrateSerial();
	void calibrateSensor();

	void setCalibrationSerial();
	void setCalibrationLocal();

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

	SerialInterface *ser;
};

#endif /* ACCGYRO_H_ */
