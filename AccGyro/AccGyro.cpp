/*
 * AccGyro.cpp
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#include "AccGyro.h"
#include <Arduino.h>
#include "Calibrator.h"

volatile bool interrupt = false;     // indicates whether MPU interrupt pin has gone high

void interruptArrived(){
	interrupt = true;
}

AccGyro::AccGyro(int devAddr): MPU6050DMP(devAddr){
	address = devAddr;
	initialized = false;
	dmpInitialized = false;
	dmpPacketSize = 42;
	mpuIntStatus = 0;
	fifoCount = 0;
	fSimulate = false;
	currentIndex = 0;
}

void AccGyro::init(){
	int dmpStatus;

	Serial.println("Initializing MPU6050 sensor");
	initialize();

	if((getDeviceID()==address) & testConnection()) initialized = true;

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	dmpStatus = dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (dmpStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, interruptArrived, RISING);
		mpuIntStatus = getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpInitialized = true;

		// get expected DMP packet size for later comparison
		dmpPacketSize = dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(dmpStatus);
		Serial.println(F(")"));
	}

	data.setFullScaleAccelerometer(getFullScaleAccelRange());
	data.setFullScaleGyroscope(getFullScaleGyroRange());
}

AccGyro::~AccGyro() {
}

void AccGyro::exportValueToSerial(){
	if(!checkDataAvailable()) return;

	VectorFloat acc, realAcc;
	VectorFloat realGyro;
	Quaternion quat;

	if(!fillValues()) return;

	acc = data.getLinearAcceleration();
	realAcc = data.getTrueAcceleration();
	realGyro = data.getAngularRate();
	quat = data.getQuaternion();

	Serial.print("Acceleration: (");
	Serial.print(acc.x);
	Serial.print(",");
	Serial.print(acc.y);
	Serial.print(",");
	Serial.print(acc.z);
	Serial.println(")");
	Serial.print("Real Acceleration: (");
	Serial.print(realAcc.x);
	Serial.print(",");
	Serial.print(realAcc.y);
	Serial.print(",");
	Serial.print(realAcc.z);
	Serial.println(")");
	Serial.print("Real Gyroscope: (");
	Serial.print(realGyro.x);
	Serial.print(",");
	Serial.print(realGyro.y);
	Serial.print(",");
	Serial.print(realGyro.z);
	Serial.println(")");
	Serial.print("Quaternion: (");
	Serial.print(quat.w);
	Serial.print(",");
	Serial.print(quat.x);
	Serial.print(",");
	Serial.print(quat.y);
	Serial.print(",");
	Serial.print(quat.z);
	Serial.println(")");
}

void AccGyro::exportTeaPot(){
	if(!checkDataAvailable()) return;

	fillValues();

	// display quaternion values in InvenSense Teapot demo format:
	Serial.write(data.getTeaPotPacket(), 14);
}

bool AccGyro::checkDataAvailable(){
	int status;

	if(fSimulate && Serial.available()>0) return true;

	if (!dmpInitialized) return false;
	if(!interrupt && fifoCount<dmpPacketSize) return false;

	fifoCount = getFIFOCount();
	if(interrupt){
		interrupt = false;
		status = getIntStatus();

		if((status & 0x10) || fifoCount==1024 ){
			//Overflow
			Serial.print("FIFO overflow. Resetting...");
			resetFIFO();
			return false;
		}
		else if(status & 0x02){
			//New data
			//Wait for complete block
			while (fifoCount < dmpPacketSize) fifoCount = getFIFOCount();
			return true;
		}
	}

	if(fifoCount>dmpPacketSize) return true;
	return false;
}

bool AccGyro::fillValues(){
	if(fSimulate) return readFromSerial();
	else return readFromSensor();
}

bool AccGyro::readFromSensor(){
	// read a packet from FIFO
	uint8_t fifoBuffer[64];

	getFIFOBytes(fifoBuffer, dmpPacketSize);
	fifoCount -= dmpPacketSize;

	data.setFromBuffer(fifoBuffer, millis());
	return true;
}

bool AccGyro::readFromSerial() {
	String s;

	s = Serial.readStringUntil('\n');
	if(currentIndex<4) buffer[currentIndex] = atof(s.c_str());
	else buffer[currentIndex] = s.toInt();
	currentIndex++;

	Serial.print(currentIndex);
	Serial.print(" ");
	Serial.println(s);
	if(currentIndex==10){
		data.setFromSerial(buffer, millis());
		currentIndex = 0;
		return true;
	}
	return false;
}

void AccGyro::calibrate() {
	Calibrator cc;
	int motorIndex=0;
	bool cont = false;
	bool calibrated = false;

	while(!calibrated){
		//cc.clearPoints();
		for(int power=100; power<1000; power += 100){
			//Set motor power
			setMotorPower(motorIndex, power);
			//Measure few values
			for(int count=0; count<5; count++){
				while(!cont){
					cont = checkDataAvailable();
					if(cont){
						cont = fillValues();
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

void AccGyro::calibrateSerial() {
}

void AccGyro::calibrateSensor() {
}

void AccGyro::setMotorPower(int motor, int power){
	Serial.print("CMD:power:");
	Serial.print(motor);
	Serial.print(":");
	Serial.println(power);
}
