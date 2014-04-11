/*
 * CalibrationLoop.cpp
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#include "CalibrationLoop.h"
#include "GenericMotor.h"
#include "GenericSensor.h"
#include <EEPROM.h>
#include "Constants.h"

CalibrationLoop::CalibrationLoop() {
	fState = kIDLE;
	fCurrentPower = 0;
	calibHeight = 50;

	fCurrentMotor = 0;
	fNextState = kIDLE;
	fIPInterval = 5;
	fITInterval = 100;

	fStopTime = 0;

	fMTInterval = 100;
	fMPInterval = 15;

	fMaxLoop = 2;
	fLoopIndex = 0;

	fPath = kPROCEDURE;
}

CalibrationLoop::~CalibrationLoop() {
}

/**
 * Setting calibration path (execute the procedure or load from memory)
 * @param p
 */
void CalibrationLoop::setCalibPath(Path p) {
	PRINTOUT("setCalibPath");
	cout << "Setting calibration path to " << p << endl;
	fPath = p;
}

/**
 * Entry point for the FSM. Determine which method should be applied as function of the state.
 * @return true if in IDLE state. Else false.
 */
bool CalibrationLoop::processLoop() {
	PRINTOUT("processLoop");
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
	PRINTOUT("scanP");
	if(!sSensor->checkDataAvailable()) return;
	double az = sSensor->getAcceleration()[2];
	//cout << "Acceleration z" << az << endl;
	if(az<=0){
		fCurrentPower++;
		sMotor->setMotorPowerAll(fCurrentPower);
	}
	else{
		sAltitude->setMotorLim(fCurrentPower);
		sAltitude->setZRef(calibHeight);
		fCurrentPower++;
		sMotor->setMotorPowerAll(fCurrentPower);
		fState = kTAKEOFF;
		cout << F("Starting take-off") << endl;
	}
}

/**
 * Keeping the value of power until height calibHeight is reached.
 */
void CalibrationLoop::takeOff() {
	PRINTOUT("takeOff");
	if(!sSensor->checkDataAvailable()) return;
	double height = sSensor->getPosition()[2];
	double vz = sSensor->getVelocity()[2];
	fCurrentPower = sAltitude->loop(height, vz);
	if(sAltitude->isStable()){
		//fState = kSTABILIZING;
		sMotor->setMotorPower(getSqrtMotorPower(1), sMotor->getFirstMotor());
		fStopTime = sSensor->getTime()+fITInterval;
		fNextState = kIDISTURBED;
		fState = kWAITING;
		cout << F("Starting waiting") << endl;
	}
	else{
		sMotor->setMotorPowerAll(fCurrentPower);
	}
}

/**
 * Decrease the value of power until the vertical acceleration is small.
 */
void CalibrationLoop::stabilize() {
	PRINTOUT("stabilize");
	if(!sSensor->checkDataAvailable()) return;
	double az = sSensor->getAcceleration()[2];
	if(az>0){
		fCurrentPower--;
		sMotor->setMotorPowerAll(fCurrentPower);
	}
	else{
		sMotor->setMotorPower(getSqrtMotorPower(1), sMotor->getFirstMotor());
		fStopTime = sSensor->getTime()+fITInterval;
		fNextState = kIDISTURBED;
		fState = kWAITING;
	}
}

/**
 * Wait until the time fStopTime is reached.
 */
void CalibrationLoop::wait() {
	PRINTOUT("wait");
	if(sSensor->getTime()>=fStopTime){
		fState = fNextState;
		cout << F("Starting ") << fNextState << endl;
	}
}

/**
 * The model is in the disturbed state (to avoid measuring at 0 rad). Ready to start the measurement.
 */
void CalibrationLoop::iDisturbed(){
	PRINTOUT("iDisturbed");
	sMotor->setMotorPower(getSqrtMotorPower(1), sMotor->getFirstMotor());
	fStopTime = sSensor->getTime()+fITInterval;
	fNextState = kIMEASUREP;
	fState = kWAITING;
	cout << F("Starting waiting") << endl;
}

/**
 * First matrix measurement.
 */
void CalibrationLoop::measureP() {
	PRINTOUT("measureP");
	if(!sSensor->checkDataAvailable()) return;
	fCalibrator.newPoint(sMotor->getFirstMotor(),
			getSqrtMotorPower(1),
			sSensor->getOmega(),
			sSensor->getAlpha(),
			sSensor->getAcceleration(),
			sSensor->getQuaternion());
	sMotor->setMotorPower(getSqrtMotorPower(-1), sMotor->getFirstMotor());
	fStopTime = sSensor->getTime()+fITInterval;
	fNextState = kIMEASUREM;
	fState = kWAITING;
}

/**
 * Second matrix measurement
 */
void CalibrationLoop::measureM() {
	PRINTOUT("measureM");
	if(!sSensor->checkDataAvailable()) return;
	fCalibrator.newPoint(sMotor->getFirstMotor(),
			getSqrtMotorPower(-1),
			sSensor->getOmega(),
			sSensor->getAlpha(),
			sSensor->getAcceleration(),
			sSensor->getQuaternion());

	//Do we restart the loop for the average?
	if(fLoopIndex<fMaxLoop){
		sMotor->setMotorPower(getSqrtMotorPower(1), sMotor->getFirstMotor());
		fStopTime = sSensor->getTime()+fITInterval;
		fNextState = kIMEASUREP;
		fState = kWAITING;
		++fLoopIndex;
	}
	else{
		fCalibrator.calibrateI(sMotor->getFirstMotor());
		fCalibrator.clearPoints();
		fCalibrator.finalizeI();
		fCalibrator.getIAxis().print();
		sMotor->setMotorPowerAll(fCurrentPower);
		sMotor->setMotorPower(getSqrtMotorPower(-1), sMotor->getFirstMotor());
		fStopTime = sSensor->getTime()+fITInterval;
		fCurrentMotor = sMotor->getFirstMotor();
		fNextState = kMDISTURBED;
		fState = kWAITING;
	}
}

void CalibrationLoop::mDisturbed() {
	PRINTOUT("mDisturbed");
	sMotor->setMotorPower(getSqrtMotorPower(1), fCurrentMotor);
	fStopTime = sSensor->getTime()+fMTInterval;
	fNextState = kMMEASURES;
	fState = kWAITING;
}

void CalibrationLoop::measureS() {
	PRINTOUT("measureS");
	if(!sSensor->checkDataAvailable()) return;
	fCalibrator.newPoint(fCurrentMotor,
			getSqrtMotorPower(1),
			sSensor->getOmega(),
			sSensor->getAlpha(),
			sSensor->getAcceleration(),
			sSensor->getQuaternion());
	sMotor->setMotorPower(getSqrtMotorPower(2), fCurrentMotor);
	fStopTime = sSensor->getTime()+fMTInterval;
	fNextState = kMMEASURED;
	fState = kWAITING;
}

void CalibrationLoop::measureD() {
	PRINTOUT("measureD");
	if(!sSensor->checkDataAvailable()) return;
	fCalibrator.newPoint(fCurrentMotor,
			getSqrtMotorPower(2),
			sSensor->getOmega(),
			sSensor->getAlpha(),
			sSensor->getAcceleration(),
			sSensor->getQuaternion());
	sMotor->setMotorPower(getSqrtMotorPower(-2), fCurrentMotor);
	fStopTime = sSensor->getTime()+fMTInterval;
	fNextState = kMBALANCED;
	fState = kWAITING;
}

void CalibrationLoop::mBalancedD(){
	PRINTOUT("mBalancedD");
	sMotor->setMotorPower(getSqrtMotorPower(-2), fCurrentMotor);
	fStopTime = sSensor->getTime()+fMTInterval;
	fNextState = kMBALANCES;
	fState = kWAITING;
}

void CalibrationLoop::mBalancedS(){
	PRINTOUT("mBalancedS");
	sMotor->setMotorPower(fCurrentPower, fCurrentMotor);
	fCalibrator.calibrateR(fCurrentMotor);
	if(fCurrentMotor==sMotor->getLastMotor()){
		for(int i=0; i<4; i++){
			//EEPROM_writeAnything<float>(eepromAddress[0+i*4], fCalibrator.getR(i)[0]);
			//EEPROM_writeAnything<float>(eepromAddress[1+i*4], fCalibrator.getR(i)[1]);
			//EEPROM_writeAnything<float>(eepromAddress[2+i*4], fCalibrator.getR(i)[2]);
			//EEPROM_writeAnything<float>(eepromAddress[3+i*4], fCalibrator.getR(i)[3]);
		}
		//EEPROM_writeAnything<float>(eepromAddress[16], fCalibrator.getIAxis().x);
		//EEPROM_writeAnything<float>(eepromAddress[17], fCalibrator.getIAxis().y);
		//EEPROM_writeAnything<float>(eepromAddress[18], fCalibrator.getIAxis().z);
		fState = kAPPLY;
	}
	else{
		fCurrentMotor++;
		fState = kMDISTURBED;
	}
}

void CalibrationLoop::load() {
	PRINTOUT("load");
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

void CalibrationLoop::start() {
	PRINTOUT("start");
	if(fPath==kPROCEDURE){
		cout << "Starting calibration procedure" << endl;
		//fCurrentPower = 1;
		fCurrentPower = 29.766;
		//fState = kSCANNING;
		sMotor->setMotorPowerAll(fCurrentPower);
		sMotor->setMotorPower(getSqrtMotorPower(1), sMotor->getFirstMotor());
		fStopTime = sSensor->getTime()+fITInterval;
		fNextState = kIDISTURBED;
		fState = kWAITING;
	}
	else fState = kLOAD;
}

void CalibrationLoop::compute() {
	PRINTOUT("compute");
}

void CalibrationLoop::apply() {
	PRINTOUT("apply");

}

double CalibrationLoop::getSqrtMotorPower(int factor) {
	return sqrt(pow(fCurrentPower,2)+factor*fIPInterval);
}
