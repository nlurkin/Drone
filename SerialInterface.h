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
	MatrixNic<float, 3, 3> cmdRequestI();

	bool checkDataAvailable();
	bool read();

	void readData(String s);
	void readCmd(String s);

	void readSensor(String s);
	void readIMat(String s);

	bool isIReady();

	MatrixNic<float, 3, 3> getI();
private:
	MatrixNic<float, 3, 3> fI;

	int fICount;
};

#endif /* SERIALINTERFACE_H_ */
