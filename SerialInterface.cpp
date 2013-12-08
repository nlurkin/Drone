/*
 * SerialInterface.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "SerialInterface.h"

SerialInterface::SerialInterface() {
	// TODO Auto-generated constructor stub
	Wire.begin();

	Serial.begin(9600);
	Serial.println("Drone");
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

MatrixNic<float, 3, 3> SerialInterface::cmdRequestI(){
	String s;
	MatrixNic<float, 3, 3> I;

	Serial.println("CMD:sendI");

	int vals = 0;
	while(vals<3){
		if(Serial.available()){
			s = Serial.readStringUntil('\n');
			I(vals, vals) = atof(s.c_str());
			Serial.println(s);
			vals++;
		}
	}
	return I;
}

bool SerialInterface::checkDataAvailable() {
}

bool SerialInterface::read(){
	bool r = false;
	String s;

	while(Serial.available()>0){
		s = Serial.readStringUntil('\n');
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
}

void SerialInterface::readIMat(String s) {
	if(s.startsWith("Ixx:")){
		fI(0,0) = atof(s.substring(4).c_str());
	}
	else if(s.startsWith("Iyy:")){
			fI(1,1) = atof(s.substring(4).c_str());
	}
	else if(s.startsWith("Izz:")){
			fI(2,2) = atof(s.substring(4).c_str());
	}
	fICount++;
}

bool SerialInterface::isIReady() {
	 return (fICount==3);
}

MatrixNic<float, 3, 3> SerialInterface::getI() {
	fICount = 0;
	return fI;
}
