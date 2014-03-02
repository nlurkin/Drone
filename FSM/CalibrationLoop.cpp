/*
 * CalibrationLoop.cpp
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#include "CalibrationLoop.h"
#include "GenericSensor.h"
#include "MotorControl.h"

CalibrationLoop::CalibrationLoop() {
	// TODO Auto-generated constructor stub
	fState = kIDLE;
	fCurrentPower = 0;
}

CalibrationLoop::~CalibrationLoop() {
	// TODO Auto-generated destructor stub
}

void CalibrationLoop::setCalibPath(Path p) {
	fPath = p;
}

bool CalibrationLoop::processLoop() {
	bool ret = false;

	switch(fState){
	case kIDLE:
		ret = true;
		break;
	case kWAITING:
		wait();
		break;
	case kSCANNING:
		scanP();
		break;
	case kTAKEOFF:
		takeOff();
		break;
	case kSTABILIZING:
		stabilize();
		break;
	case kIDISTURBED:
		disturbed();
		break;
	case kIMEASUREP:
		measureP();
		break;
	case kIMEASUREM:
		measureM();
		break;
	default:
		//Should not happen
		break;
	}

	return ret;
}

void CalibrationLoop::scanP(){
	double az = fSensor->getAcceleration()[2];
	if(az<=0){
		fCurrentPower++;
		fMotorControl->setMotorPowerAll(fCurrentPower);
	}
	else{
		fState = kTAKEOFF;
	}
}

void CalibrationLoop::takeOff() {
	double height = fSensor->getPosition()[2];
	if(height == calibHeight){
		fState = kSTABILIZING;
	}
}

void CalibrationLoop::stabilize() {
	double az = fSensor->getAcceleration()[2];
	if(az>0){
		fCurrentPower--;
		fMotorControl->setMotorPowerAll(fCurrentPower);
	}
	else{
		fMotorControl->setMotorPower(fCurrentPower+fIPInterval, fMotorControl->getFirstMotor());
		fStopTime = millis()+fITInterval;
		fNextState = kIDISTURBED;
		fState = kWAITING;
	}
}

void CalibrationLoop::wait() {
	if(millis()>=fStopTime) fState = fNextState;
}

void CalibrationLoop::disturbed(){
	fMotorControl->setMotorPower(fCurrentPower+fIPInterval, fMotorControl->getFirstMotor());
	fStopTime = millis()+fITInterval;
	fNextState = kIMEASUREP;
	fState = kWAITING;
}

void CalibrationLoop::measureP() {
	fCalibrator.newPoint(fMotorControl->getFirstMotor(),
			fCurrentPower+fIPInterval,
			fSensor->getOmega(),
			fSensor->getAlpha(),
			fSensor->getAcceleration(),
			fSensor->getQuaternion());
	fMotorControl->setMotorPower(fCurrentPower-fIPInterval, fMotorControl->getFirstMotor());
	fStopTime = millis()+fITInterval;
	fNextState = kIMEASUREM;
	fState = kWAITING;
}

void CalibrationLoop::measureM() {
	fCalibrator.newPoint(fMotorControl->getFirstMotor(),
			fCurrentPower-fIPInterval,
			fSensor->getOmega(),
			fSensor->getAlpha(),
			fSensor->getAcceleration(),
			fSensor->getQuaternion());
	if(fLoopIndex<fMaxLoop){
		fMotorControl->setMotorPower(fCurrentPower+fIPInterval, fMotorControl->getFirstMotor());
		fStopTime = millis()+fITInterval;
		fNextState = kIMEASUREP;
		fState = kWAITING;
	}
	else{
		fMotorControl->setMotorPower(fCurrentPower-fIPInterval, fMotorControl->getFirstMotor());
		fStopTime = millis()+fITInterval;
		fNextState = kMDISTURBED;
		fState = kWAITING;
	}
}
