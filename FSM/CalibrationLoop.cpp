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
	fState = kIDLE;
	fCurrentPower = 0;
}

CalibrationLoop::~CalibrationLoop() {
}

/**
 * Setting calibration path (execute the procedure or load from memory)
 * @param p
 */
void CalibrationLoop::setCalibPath(Path p) {
	fPath = p;
}

/**
 * Entry point for the FSM. Determine which method should be applied as function of the state.
 * @return true if in IDLE state. Else false.
 */
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

/**
 * Scanning value of power until vertical linear acceleration is positive.
 */
void CalibrationLoop::scanP(){
	if(!fSensor->checkDataAvailable()) return;
	double az = fSensor->getAcceleration()[2];
	if(az<=0){
		fCurrentPower++;
		fMotorControl->setMotorPowerAll(fCurrentPower);
	}
	else{
		fState = kTAKEOFF;
	}
}

/**
 * Keeping the value of power until height calibHeight is reached.
 */
void CalibrationLoop::takeOff() {
	if(!fSensor->checkDataAvailable()) return;
	double height = fSensor->getPosition()[2];
	if(height == calibHeight){
		fState = kSTABILIZING;
	}
}

/**
 * Decrease the value of power until the vertical acceleration is small.
 */
void CalibrationLoop::stabilize() {
	if(!fSensor->checkDataAvailable()) return;
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

/**
 * Wait until the time fStopTime is reached.
 */
void CalibrationLoop::wait() {
	if(millis()>=fStopTime) fState = fNextState;
}

/**
 * The model is in the disturbed state (to avoid measuring at 0 rad). Ready to start the measurement.
 */
void CalibrationLoop::iDisturbed(){
	fMotorControl->setMotorPower(fCurrentPower+fIPInterval, fMotorControl->getFirstMotor());
	fStopTime = millis()+fITInterval;
	fNextState = kIMEASUREP;
	fState = kWAITING;
}

/**
 * First matrix measurement.
 */
void CalibrationLoop::measureP() {
	if(!fSensor->checkDataAvailable()) return;
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

/**
 * Second matrix measurement
 */
void CalibrationLoop::measureM() {
	if(!fSensor->checkDataAvailable()) return;
	fCalibrator.newPoint(fMotorControl->getFirstMotor(),
			fCurrentPower-fIPInterval,
			fSensor->getOmega(),
			fSensor->getAlpha(),
			fSensor->getAcceleration(),
			fSensor->getQuaternion());

	//Do we restart the loop for the average?
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
	if(!fSensor->checkDataAvailable()) return;
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
	if(!fSensor->checkDataAvailable()) return;
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
			EEPROM_writeAnything<float>(eepromAddress[0+i*4], fCalibrator.getR(i)[0]);
			EEPROM_writeAnything<float>(eepromAddress[1+i*4], fCalibrator.getR(i)[1]);
			EEPROM_writeAnything<float>(eepromAddress[2+i*4], fCalibrator.getR(i)[2]);
			EEPROM_writeAnything<float>(eepromAddress[3+i*4], fCalibrator.getR(i)[3]);
		}
		EEPROM_writeAnything<float>(eepromAddress[16], fCalibrator.getIAxis().x);
		EEPROM_writeAnything<float>(eepromAddress[17], fCalibrator.getIAxis().y);
		EEPROM_writeAnything<float>(eepromAddress[18], fCalibrator.getIAxis().z);
		fState = kAPPLY;
	}
	else{
		fCurrentMotor++;
		fState = kMDISTURBED;
	}
}

void CalibrationLoop::load() {
	for(int i=0; i<4; i++){
		EEPROM_readAnything<float>(eepromAddress[0+i*4], fCalibrator.getR(i)[0]);
		EEPROM_readAnything<float>(eepromAddress[1+i*4], fCalibrator.getR(i)[1]);
		EEPROM_readAnything<float>(eepromAddress[2+i*4], fCalibrator.getR(i)[2]);
		EEPROM_readAnything<float>(eepromAddress[3+i*4], fCalibrator.getR(i)[3]);
	}
	EEPROM_readAnything<float>(eepromAddress[16], fCalibrator.getIAxis()[0]);
	EEPROM_readAnything<float>(eepromAddress[17], fCalibrator.getIAxis()[1]);
	EEPROM_readAnything<float>(eepromAddress[18], fCalibrator.getIAxis()[2]);
}

void CalibrationLoop::apply() {

}
