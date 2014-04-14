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
#include "GenericSensor.h"
#include "GenericMotor.h"
#include "GenericControl.h"
#include "AccGyroData.h"
#include "Constants.h"

class SerialInterface: public GenericSensor, public GenericMotor, public GenericControl {
public:
	SerialInterface();
	virtual ~SerialInterface();

	Quaternion getQuaternion();
	VectorFloat getOmega();
	VectorFloat getAcceleration();
	VectorFloat getPosition();
	VectorFloat getAlpha();
	VectorFloat getVelocity();

	int getTime();

	Constants::CtrlCommand::ECtrlCommand getCtrlCommand();

	bool checkDataAvailable();

	bool isIReady();
	bool isSensorReady();
	bool isAttitudeReady();
	bool isCtrlCommandReady();
	bool isSimpleKFactorsReady();

	void disableAll();
	void setMotorPowerAll(double power);
	void setMotorPower(double power, int i);

	int getFirstMotor();
	int getLastMotor();
	double getSimpleKP();
	double getSimpleKD();

	bool read();
	void cmdNextStep();

private:
	void cmdPower(int motor, double power);
	void cmdRequestI();
	void cmdRequestData();
	void cmdRequestTime();
	void cmdTorque(VectorFloat tau);

	void readData(String s);
	void readCmd(String s);

	void readSensor(String s);
	void readIMat(String s);
	void readTime(String s);
	void readNewAttitude(String s);
	void readCtrlCommand(String s);
	void readKValues(String s);

	VectorFloat getI();
	float* getBuffer();

	VectorFloat fI;
	float fBuffer[10];
	int fTime;
	Quaternion fRefQuat;
	AccGyroData fData;

	int fICount;
	int fBufferCount;
	int fQuatCount;

	int fFirstMotor;
	int fLastMotor;

	double fSimKP, fSimKD;
	int fSimKCount;

	Constants::CtrlCommand::ECtrlCommand fCtrlCommand;
	bool fCtrlCommandReady;

	int fRequests;
};

#endif /* SERIALINTERFACE_H_ */
