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

	bool isIReady();
	bool isSensorReady();

	MatrixNic<float, 3, 3> getI();
	float* getBuffer();
private:
	MatrixNic<float, 3, 3> fI;
	float fBuffer[10];

	int fICount;
	int fBufferCount;
};

#endif /* SERIALINTERFACE_H_ */
