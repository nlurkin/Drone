/*
 * SerialInterface.h
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#ifndef SERIALINTERFACE_H_
#define SERIALINTERFACE_H_

#include <Wire.h>
#include <Arduino.h>
#include "mathClasses.h"

class SerialInterface: public GenericSensor, public MotorControl {
public:
	SerialInterface();
	virtual ~SerialInterface();

	Quaternion getQuaternion();
	VectorFloat getOmega();
	VectorFloat getAcceleration();
	VectorFloat getPosition();
	VectorFloat getAlpha();

	bool checkDataAvailable();

	void disableAll();
	void setMotorPowerAll(double power);
	void setMotorPower(double power, int i);

	int getFirstMotor();
	int getLastMotor();

private:
	void cmdPower(int motor, int power);
	void cmdRequestI();
	void cmdTorque(VectorFloat tau);

	bool read();

	void readData(String s);
	void readCmd(String s);

	void readSensor(String s);
	void readIMat(String s);
	void readNewAttitude(String s);

	bool isIReady();
	bool isSensorReady();
	bool isAttitudeReady();

	VectorFloat getI();
	float* getBuffer();

	VectorFloat fI;
	float fBuffer[10];
	Quaternion fRefQuat;
	AccGyroData fData;

	int fICount;
	int fBufferCount;
	int fQuatCount;

	int fFirstMotor;
	int fLastMotor;
};

#endif /* SERIALINTERFACE_H_ */
