/*
 * MainLoop.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "MainLoop.h"
#include <Arduino.h>
#include "Calibrator.h"
#include "Constants.h"
#include "SerialInterface.h"
#include "GenericControl.h"
#include "SerialOutput.h"

MainLoop::MainLoop(){
	// TODO Auto-generated constructor stub
	//fSensor = new AccGyro(0x68);
	sControl = new SerialInterface();
	sMotor = (SerialInterface*)sControl;
	sSensor = (SerialInterface*)sControl;
	//cout = new SerialOutput();

	fSimulate = false;

	fState = kIDLE;

	fCtrlCommand = Constants::CtrlCommand::kNONE;
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
	sMotor->disableAll();
}

void MainLoop::loop(){
	sControl->read();
	if(sControl->isCtrlCommandReady()){
		fCtrlCommand = sControl->getCtrlCommand();
		cout << "Control command received " << (int)fCtrlCommand << SerialOutput::endl;
		delay(100);
		if(fCtrlCommand==Constants::CtrlCommand::kDODEBUG) bigDebug = !bigDebug;
	}
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
	cout << "Moving to IDLE" << SerialOutput::endl;
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fState = kIDLE;
}
void MainLoop::moveToCalibration() {
	cout << "Moving to CALIBRATION" << SerialOutput::endl;
	delay(100);
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fLoopCalib.setCalibPath(CalibrationLoop::kPROCEDURE);
	fLoopCalib.start();
	fState = kCALIBRATING;
}
void MainLoop::moveToFlight() {
	cout << "Moving to FLIGHT" << SerialOutput::endl;
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fState = kFLYING;
}
