/*
 * SerialInterface.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "SerialInterface.h"

SerialInterface::SerialInterface() {
	// TODO Auto-generated constructor stub
	Serial.println("Drone serial interface initialized");
	fICount = 0;
	fBufferCount = 0;
}

SerialInterface::~SerialInterface() {
	// TODO Auto-generated destructor stub
}

void SerialInterface::cmdPower(int motor, int power) {
	Serial.print("CMD:power:");
	Serial.print(motor);
	Serial.print(":");
	Serial.println(power);
}

void SerialInterface::cmdRequestI(){
	Serial.println("CMD:sendI");
}

bool SerialInterface::read(){
	String s;
	bool r = false;

	while(Serial.available()>0){
		s = Serial.readStringUntil('\n');
		Serial.print("Receiving from Serial: ");
		Serial.println(s);
		//Does it look like a command
		if(s.startsWith("CMD:")){
			readCmd(s.substring(4));
		}
		//Does it look like Data
		else if(s.startsWith("DAT:")){
			readData(s.substring(4));
		}
	}
	return r;
}

void SerialInterface::readData(String s) {
	if(s.startsWith("SENS:")){
		readSensor(s.substring(5));
	}
	else if(s.startsWith("IMAT:")){
		readIMat(s.substring(5));
	}
}

void SerialInterface::readCmd(String s) {
}

void SerialInterface::readSensor(String s) {
	if(fBufferCount==10) fBufferCount=0;

	if(s.startsWith("BUF0:")) fBuffer[0] = atof(s.substring(5).c_str());
	else if(s.startsWith("BUF1:")) fBuffer[1] = atof(s.substring(5).c_str());
	else if(s.startsWith("BUF2:")) fBuffer[2] = atof(s.substring(5).c_str());
	else if(s.startsWith("BUF3:")) fBuffer[3] = atof(s.substring(5).c_str());
	else if(s.startsWith("BUF4:")) fBuffer[4] = s.substring(5).toInt();
	else if(s.startsWith("BUF5:")) fBuffer[5] = s.substring(5).toInt();
	else if(s.startsWith("BUF6:")) fBuffer[6] = s.substring(5).toInt();
	else if(s.startsWith("BUF7:")) fBuffer[7] = s.substring(5).toInt();
	else if(s.startsWith("BUF8:")) fBuffer[8] = s.substring(5).toInt();
	else if(s.startsWith("BUF9:")) fBuffer[9] = s.substring(5).toInt();
	fBufferCount++;
	if(fBufferCount==10) Serial.println("Full buffer received");
}

void SerialInterface::readIMat(String s) {
	if(fICount==3) fICount=0;

	if(s.startsWith("IXX:")) fI(0,0) = atof(s.substring(4).c_str());
	else if(s.startsWith("IYY:")) fI(1,1) = atof(s.substring(4).c_str());
	else if(s.startsWith("IZZ:")) fI(2,2) = atof(s.substring(4).c_str());
	fICount++;
	if(fICount==3) Serial.println("Full IMat received");
}

bool SerialInterface::isIReady() {
	return (fICount==3);
}

MatrixNic<float, 3, 3> SerialInterface::getI() {
	fICount = 0;
	return fI;
}

bool SerialInterface::isSensorReady() {
	return (fBufferCount==10);
}

void SerialInterface::cmdTorque(VectorFloat tau) {
	Serial.print("CMD:TAUS:TAUX:");
	Serial.println(tau.x);
	Serial.print("CMD:TAUS:TAUY:");
	Serial.println(tau.y);
	Serial.print("CMD:TAUS:TAUZ:");
	Serial.println(tau.z);
}

float* SerialInterface::getBuffer() {
	fBufferCount = 0;
	for(int i=0; i<10; i++){
		Serial.print(fBuffer[i]);
		Serial.print(" ");
	}
	Serial.println("");
	return fBuffer;
}
