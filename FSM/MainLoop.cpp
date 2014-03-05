/*
 * MainLoop.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "MainLoop.h"
#include <Arduino.h>
#include "Calibrator.h"
#include "MotorControl.h"
#include "GenericSensor.h"

MainLoop::MainLoop(){
	// TODO Auto-generated constructor stub
	//fSensor = new AccGyro(0x68);
	fSerial = new SerialInterface();
	fMotorCtrl = fSerial;
	fSensor = fSerial;

	fSimulate = false;
	//fCalibrated = false;
	//fCalibrationRequested = false;

	fState = kIDLE;
}

MainLoop::~MainLoop() {
	// TODO Auto-generated destructor stub
}

void MainLoop::setup(){
	fSimulate = true;
	//fSensor->setSerialInterface(fSerial);
	//fSensor->setSimulate(true);
	//ctl.setQRef(Quaternion(1, 0, 0, 0));
	//ctl.setP(20, 4);

	//Reset all motors
	fMotorCtrl->disableAll();
}

void MainLoop::loop(){
	fSerial->read();
	if(fSerial->isCtrlCommandReady()) fCtrlCommand = fSerial->getCtrlCommand();
	if(fState==kINITIALIZING) initializationLoop();
	else if(fState==kIDLE) idleLoop();
	else if(fState==kCALIBRATING) calibrationLoop();
	else if(fState==kFLYING) flightLoop();
	else{
		//Error, do something
	}
}

void MainLoop::initializationLoop(){
	moveToIdle();
}

void MainLoop::idleLoop(){
	if(fCtrlCommand==Constants::CtrlCommand::kDOCALIB) moveToCalibration();
}

void MainLoop::calibrationLoop(){
	if(fLoopCalib.processLoop()) moveToIdle();
}

void MainLoop::flightLoop() {
	/*VectorFloat tau;
	if(ser.isAttitudeReady()){
		ctl.setQRef(ser.getAttitude());
	}
	if(sensor.checkDataAvailable()){
		if(sensor.fillValues()){
			sensor.exportValueToSerial();
			tau = ctl.ComputePP(sensor.getQuaternion(), sensor.getOmega());
			ser.cmdTorque(tau);
		}
	}*/
}



void MainLoop::moveToIdle() {
	fState = kIDLE;
}
void MainLoop::moveToCalibration() {
	fLoopCalib.setCalibPath(CalibrationLoop::kPROCEDURE);
	fState = kCALIBRATING;
}
void MainLoop::moveToFlight() {
	fState = kFLYING;
}
