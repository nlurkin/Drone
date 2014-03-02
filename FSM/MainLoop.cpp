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
	fSensor = new AccGyro(0x68);
	fMotorCtrl = new MotorControl();
	fSerial = new SerialInterface();

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
	fSensor->setSerialInterface(fSerial);
	//fSensor->setSimulate(true);
	//ctl.setQRef(Quaternion(1, 0, 0, 0));
	//ctl.setP(20, 4);

	//Reset all motors
	fMotorCtrl->disableAll();
}

void MainLoop::loop(){
	fSerial->read();
	if(fState==kINITIALIZING) initializationLoop();
	else if(fState==kIDLE) idleLoop();
	else if(fState==kCALIBRATING) calibrationLoop();
	else if(fState==kFLYING) flightLoop();
	else{
		//Error, do something
	}
}

void MainLoop::initializationLoop(){
}

void MainLoop::idleLoop(){
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

/*
void MainLoop::setCalibration() {
	if(fSimulate) setCalibrationSerial();
	else setCalibrationLocal();
}

void MainLoop::setCalibrationSerial() {
	if(!fCalibrationRequested){
		ser.cmdRequestI();
		fCalibrationRequested = true;
	}
	if(ser.isIReady()){
		ctl.setI(ser.getI());
		fCalibrated = true;
	}
}

void MainLoop::setCalibrationLocal() {
}

void MainLoop::calibrateSerial() {
}

void MainLoop::calibrateSensor() {
}*/

/*void MainLoop::calibrate() {
	Calibrator cc;
	int motorIndex=0;
	bool cont = false;
	bool calibrated = false;

	while(!calibrated){
		//cc.clearPoints();
		for(int power=100; power<1000; power += 100){
			//Set motor power
			ser.cmdPower(motorIndex, power);
			//Measure few values
			for(int count=0; count<5; count++){
				while(!cont){
					cont = sensor.checkDataAvailable();
					if(cont){
						cont = sensor.fillValues();
					}
				}
				cont = false;
				Serial.println(count);
			}

			//Add new point
			Serial.print("New point");
			//calibrated = cc.newPoint(motorIndex, power, data.getAngularRate(), data.getAlpha());
			Serial.println(calibrated);
		}
	}
}
 */

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
