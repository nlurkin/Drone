/*
 * MainLoop.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "MainLoop.h"
#include <Arduino.h>
#include "Calibrator.h"

MainLoop::MainLoop(): sensor(0X68) {
	// TODO Auto-generated constructor stub
	fSimulate = false;
	fCalibrated = false;
	fInitialized = false;
}

MainLoop::~MainLoop() {
	// TODO Auto-generated destructor stub
}

void MainLoop::setup(){
	fSimulate = true;
	sensor.setSerialInterface(ser);
	sensor.setSimulate(true);
}

void MainLoop::loop(){
	if(!fInitialized){
		initLoop();
	}
}

void MainLoop::initLoop(){
	if(!fCalibrated) setCalibration();
	else{
		ctl.printI();
	}
}

void MainLoop::setCalibration() {
	if(fSimulate) setCalibrationSerial();
	else setCalibrationLocal();
}

void MainLoop::setCalibrationSerial() {
	if(!fCalibrationRequested) ser.cmdRequestI();
	ser.read();
	if(ser.isIReady()){
		ctl.setI(ser.getI());
	}
}

void MainLoop::setCalibrationLocal() {
}

void MainLoop::calibrateSerial() {
}

void MainLoop::calibrateSensor() {
}

void MainLoop::calibrate() {
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
