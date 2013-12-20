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

class SerialInterface {
public:
	SerialInterface();
	virtual ~SerialInterface();

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

	MatrixNic<float, 3, 3> getI();
	float* getBuffer();
	Quaternion getAttitude();
private:
	MatrixNic<float, 3, 3> fI;
	float fBuffer[10];
	Quaternion fQuat;

	int fICount;
	int fBufferCount;
	int fQuatCount;
};

#endif /* SERIALINTERFACE_H_ */
