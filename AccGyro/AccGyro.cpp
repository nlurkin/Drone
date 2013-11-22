/*
 * AccGyro.cpp
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#include "AccGyro.h"
#include <Arduino.h>

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
}

AccGyro::~AccGyro() {
}

void AccGyro::exportValueToSerial(){
	if(!checkDataAvailable()) return;

	Quaternion q;
	AccGyroData d;

	d = getValues();

		// display quaternion values in easy matrix form: w x y z
		//dmpGetQuaternion(&q, fifoBuffer);
		//dmpGetEuler(euler, &q);
		//dmpGetGravity(&gravity, &q);
		//dmpGetYawPitchRoll(ypr, &q, &gravity);
		//dmpGetAccel(&aa, fifoBuffer);
		//dmpGetLinearAccel(&aaReal, &aa, &gravity);
		//dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

		/*Serial.print("quat\t");
		Serial.print(q.w);
		Serial.print("\t");
		Serial.print(q.x);
		Serial.print("\t");
		Serial.print(q.y);
		Serial.print("\t");
		Serial.println(q.z);

		// display Euler angles in degrees
		Serial.print("euler\t");
		Serial.print(euler[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(euler[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(euler[2] * 180/M_PI);

		// display Euler angles in degrees
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180/M_PI);

		// display real acceleration, adjusted to remove gravity
		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.print(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);

		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
		Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);
		 */
		int16_t x,y,z;
		uint8_t r;
		double rx,ry,rz;
		float fsr;
		r = getFullScaleAccelRange();
		if(r==0) fsr = 8192.0 * 2;
		if(r==1) fsr = 4096.0 * 2;
		if(r==2) fsr = 2048.0 * 2;
		if(r==3) fsr = 1024.0 * 2;

		getAcceleration(&x, &y, &z);

		rx = (float)x/fsr;
		ry = (float)y/fsr;
		rz = (float)z/fsr;

		//Serial.print("Accelerator values: ");
		//Serial.print(" x=");
		Serial.print(rx);
		Serial.print(",");
		Serial.print(ry);
		Serial.print(",");
		Serial.println(rz);
	//}

	//delay(1000);
}

bool AccGyro::checkDataAvailable(){
	int status;
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

}

AccGyroData AccGyro::getValues(){
	// read a packet from FIFO
	uint8_t fifoBuffer[64];

	getFIFOBytes(fifoBuffer, dmpPacketSize);
	fifoCount -= dmpPacketSize;

}
