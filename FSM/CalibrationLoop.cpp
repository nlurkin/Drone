/*
 * CalibrationLoop.cpp
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#include "CalibrationLoop.h"
#include "GenericSensor.h"
#include "MotorControl.h"
#include <EEPROM.h>

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
		iDisturbed();
		break;
	case kIMEASUREP:
		measureP();
		break;
	case kIMEASUREM:
		measureM();
		break;
	case kMDISTURBED:
		mDisturbed();
		break;
	case kMMEASURES:
		measureS();
		break;
	case kMMEASURED:
		measureD();
		break;
	case kMBALANCES:
		mBalancedS();
		break;
	case kMBALANCED:
		mBalancedD();
		break;
	case kLOAD:
		load();
		break;
	case kAPPLY:
		apply();
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

void CalibrationLoop::iDisturbed(){
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
		fCalibrator.calibrateI(fMotorControl->getFirstMotor());
		fCalibrator.clearPoints();
		fMotorControl->setMotorPower(fCurrentPower-fIPInterval, fMotorControl->getFirstMotor());
		fStopTime = millis()+fITInterval;
		fCurrentMotor = fMotorControl->getFirstMotor();
		fNextState = kMDISTURBED;
		fState = kWAITING;
	}
}

void CalibrationLoop::mDisturbed() {
	fMotorControl->setMotorPower(fCurrentPower+fMPInterval, fCurrentMotor);
	fStopTime = millis()+fMTInterval;
	fNextState = kMMEASURES;
	fState = kWAITING;
}

void CalibrationLoop::measureS() {
	fCalibrator.newPoint(fCurrentMotor,
			fCurrentPower+fMPInterval,
			fSensor->getOmega(),
			fSensor->getAlpha(),
			fSensor->getAcceleration(),
			fSensor->getQuaternion());
	fMotorControl->setMotorPower(fCurrentPower+2*fMPInterval, fCurrentMotor);
	fStopTime = millis()+fMTInterval;
	fNextState = kMMEASURED;
	fState = kWAITING;
}

void CalibrationLoop::measureD() {
	fCalibrator.newPoint(fCurrentMotor,
			fCurrentPower+2*fMPInterval,
			fSensor->getOmega(),
			fSensor->getAlpha(),
			fSensor->getAcceleration(),
			fSensor->getQuaternion());
	fMotorControl->setMotorPower(fCurrentPower-2*fMPInterval, fCurrentMotor);
	fStopTime = millis()+fMTInterval;
	fNextState = kMBALANCED;
	fState = kWAITING;
}

void CalibrationLoop::mBalancedD(){
	fMotorControl->setMotorPower(fCurrentPower-2*fMPInterval, fCurrentMotor);
	fStopTime = millis()+fMTInterval;
	fNextState = kMBALANCES;
	fState = kWAITING;
}

void CalibrationLoop::mBalancedS(){
	fCalibrator.calibrateR(fCurrentMotor);
	if(fCurrentMotor==fMotorControl->getLastMotor()){
		for(int i=0; i<4; i++){
			EEPROM_writeAnything(eepromAddress[0+i*4], fCalibrator.fR[i].Rx);
			EEPROM_writeAnything(eepromAddress[1+i*4], fCalibrator.fR[i].Ry);
			EEPROM_writeAnything(eepromAddress[2+i*4], fCalibrator.fR[i].Rz);
			EEPROM_writeAnything(eepromAddress[3+i*4], fCalibrator.fR[i].Rt);
		}
		EEPROM_writeAnything(eepromAddress[16], fCalibrator.fIAxis.x);
		EEPROM_writeAnything(eepromAddress[17], fCalibrator.fIAxis.y);
		EEPROM_writeAnything(eepromAddress[18], fCalibrator.fIAxis.z);
		fState = kAPPLY;
	}
	else{
		fCurrentMotor++;
		fState = kMDISTURBED;
	}
}

void CalibrationLoop::load() {
	for(int i=0; i<4; i++){
		EEPROM_readAnything(eepromAddress[0+i*4], fCalibrator.fR[i].Rx);
		EEPROM_readAnything(eepromAddress[1+i*4], fCalibrator.fR[i].Ry);
		EEPROM_readAnything(eepromAddress[2+i*4], fCalibrator.fR[i].Rz);
		EEPROM_readAnything(eepromAddress[3+i*4], fCalibrator.fR[i].Rt);
	}
	EEPROM_readAnything(eepromAddress[16], fCalibrator.fIAxis.x);
	EEPROM_readAnything(eepromAddress[17], fCalibrator.fIAxis.y);
	EEPROM_readAnything(eepromAddress[18], fCalibrator.fIAxis.z);
}

void CalibrationLoop::apply() {

}
