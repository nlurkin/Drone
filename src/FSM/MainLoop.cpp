/*
 * MainLoop.cpp
 *
 *  Created on: 8 d�c. 2013
 *      Author: Nicolas
 */

#include "FSM/MainLoop.h"
#include <arduino/Arduino.h>
#include "Control/Calibrator.h"
#include "Constants.h"
#include "Communication/SerialInterface.h"
#include "Generic/GenericControl.h"
#include "Communication/SerialOutput.h"

MainLoop::MainLoop(){
	// TODO Auto-generated constructor stub
	//fSensor = new AccGyro(0x68);
	sControl = new SerialInterface();
	sMotor = (SerialInterface*)sControl;
	sSensor = (SerialInterface*)sControl;
	sAltitude = new SimpleControl();
	//cout = new SerialOutput();

	fSimulate = false;

	fState = kIDLE;

	fCtrlCommand = Constants::CtrlCommand::kNONE;
}

MainLoop::~MainLoop() {
	// TODO Auto-generated destructor stub
}

void MainLoop::start(){
	fSimulate = true;
	//fSensor->setSerialInterface(fSerial);
	//fSensor->setSimulate(true);
	//ctl.setQRef(Quaternion(1, 0, 0, 0));
	//ctl.setP(20, 4);

	//Reset all motors
	//sMotor->disableAll();
}

bool MainLoop::processLoop(){
	while(sControl->read()){};
	if(sControl->isCtrlCommandReady()){
		fCtrlCommand = sControl->getCtrlCommand();
		cout << "Control command received " << (int)fCtrlCommand << endl;
		delay(100);
		if(fCtrlCommand==Constants::CtrlCommand::kDODEBUG) bigDebug = !bigDebug;
	}
	if(sControl->isSimpleKFactorsReady()){
		sAltitude->setKs(sControl->getSimpleKP(), sControl->getSimpleKD());
	}
	if(fState==kINITIALIZING) initializationLoop();
	else if(fState==kIDLE) idleLoop();
	else if(fState==kCALIBRATING) calibrationLoop();
	else if(fState==kFLYING) flightLoop();
	else if(fState==kSTILL) stillLoop();
	else{
		//Error, do something
	}
	return true;
}

void MainLoop::initializationLoop(){
	moveToIdle();
}

void MainLoop::idleLoop(){
	if(fCtrlCommand==Constants::CtrlCommand::kDOCALIB) moveToCalibration();
	if(fCtrlCommand==Constants::CtrlCommand::kGOSTILL) moveToStill();
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

void MainLoop::stillLoop() {
	if(fLoopStill.processLoop()) moveToIdle();
}



void MainLoop::moveToIdle() {
	cout << "Moving to IDLE" << endl;
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fState = kIDLE;
}
void MainLoop::moveToCalibration() {
	cout << "Moving to CALIBRATION" << endl;
	delay(100);
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fLoopCalib.setCalibPath(CalibrationLoop::kPROCEDURE);
	fLoopCalib.start();
	fState = kCALIBRATING;
}
void MainLoop::moveToFlight() {
	cout << "Moving to FLIGHT" << endl;
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fState = kFLYING;
}
void MainLoop::moveToStill() {
	cout << "Moving to STILL" << endl;
	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fState = kSTILL;
	fLoopStill.start();
}

void MainLoop::useDefaultCalib(){
	cout << "Setting default calibration" << endl;
	MatrixNic<float, 3, 3> I;
	I(0,0) = 0.177;
	I(1,1) = 0.177;
	I(2,2) = 0.334;
	sAttitude->setI(I);
	sAttitude->setM(0, 0.0015137785799771912, 0, 3.006602344255857, 0.00398362784204524);
	sAttitude->setM(1, 0, 0.0015137785799771912, 3.006602344255857, 0.00398362784204524);
	sAttitude->setM(2, 0.0015137785799771912, 0, 3.006602344255857, 0.00398362784204524);
	sAttitude->setM(3, 0, 0.0015137785799771912, 3.006602344255857, 0.00398362784204524);
}
