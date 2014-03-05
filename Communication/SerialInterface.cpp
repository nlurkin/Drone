/*
 * SerialInterface.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "SerialInterface.h"

SerialInterface::SerialInterface() {
	Serial.println("Drone serial interface initialized");
	fICount = 0;
	fBufferCount = 0;
	fQuatCount = 0;
}

SerialInterface::~SerialInterface() {
}

/**
 * Command sent to change motor power
 * @param motor: index of the motor
 * @param power: new power
 */
void SerialInterface::cmdPower(int motor, int power) {
	Serial.print("CMD:power:");
	Serial.print(motor);
	Serial.print(":");
	Serial.println(power);
}

/**
 * Command to request the simulator to send the I matrix
 */
void SerialInterface::cmdRequestI(){
	Serial.println("CMD:sendI");
}

/**
 * Loop method. Check if something to be read on pipe and parse it.
 * @return false
 */
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

/**
 * Parse a data line.
 * @param s: data line
 */
void SerialInterface::readData(String s) {
	if(s.startsWith("SENS:")){		//Sensor values
		readSensor(s.substring(5));
	}
	else if(s.startsWith("IMAT:")){	//I Matrix values
		readIMat(s.substring(5));
	}
}

/**
 * Parse command line.
 * @param s: command line
 */
void SerialInterface::readCmd(String s) {
	if(s.startsWith("TRCK:")){				//Setting a new attitude reference
		readNewAttitude(s.substring(5));
	}
	if(s.startsWith("CTRL:")){				//Control command
		readCtrlCommand(s.substring(5));
	}
}

/**
 * Parse a sensor line. Waiting 10 values.
 * @param s: sensor line
 */
void SerialInterface::readSensor(String s) {
	if(fBufferCount==10) fBufferCount=0;

	if(s.startsWith("BUF0:")) {
		fBuffer[0] = atof(s.substring(5).c_str());
		fBufferCount=0;
	}
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
	if(fBufferCount==10){
		Serial.println("Full buffer received");
		fData.setFromSerial(fBuffer, millis());
	}
}

/**
 * Parse a I matrix line. Waiting 3 values.
 * @param s: matrix line
 */
void SerialInterface::readIMat(String s) {
	if(fICount==3) fICount=0;

	if(s.startsWith("IXX:")){
		fI[0] = atof(s.substring(4).c_str());
		fICount=0;
	}
	else if(s.startsWith("IYY:")) fI[1] = atof(s.substring(4).c_str());
	else if(s.startsWith("IZZ:")) fI[2] = atof(s.substring(4).c_str());
	fICount++;
	if(fICount==3) Serial.println("Full IMat received");
}

/**
 * Did we receive the full I matrix?
 * @return true if full matrix received.
 */
bool SerialInterface::isIReady() {
	return (fICount==3);
}

/**
 *
 * @return I matrix
 */
VectorFloat SerialInterface::getI() {
	fICount = 0;
	return fI;
}

/**
 * Did we receive a full sensor buffer?
 * @return true if full sensor buffer received.
 */
bool SerialInterface::isSensorReady(){
	//TODO faire comme ça ou pas?
	bool isReady = (fBufferCount==10);
	fBufferCount = 0;
	return false;
}

/**
 * Sending a command to change the current applied torque.
 * @param tau: torque
 */
void SerialInterface::cmdTorque(VectorFloat tau) {
	Serial.print("CMD:TAUS:TAUX:");
	Serial.println(tau.x);
	Serial.print("CMD:TAUS:TAUY:");
	Serial.println(tau.y);
	Serial.print("CMD:TAUS:TAUZ:");
	Serial.println(tau.z);
}

/**
 * Parse the new attitude quaternion line.
 * @param s: attitude quaternion line
 */
void SerialInterface::readNewAttitude(String s) {
	if(fQuatCount==4) fQuatCount=0;

	if(s.startsWith("QUAW:")){
		fRefQuat.w = atof(s.substring(5).c_str());
		fQuatCount=0;
	}
	else if(s.startsWith("QUAX:")) fRefQuat.x = atof(s.substring(5).c_str());
	else if(s.startsWith("QUAY:")) fRefQuat.y = atof(s.substring(5).c_str());
	else if(s.startsWith("QUAZ:")) fRefQuat.z = atof(s.substring(5).c_str());
	fQuatCount++;
	if(fQuatCount==4) Serial.println("Full attitude quaternion received");
}

/**
 * Did we receive a full new attitude quaternion?
 * @return true if full new attitude quaternion received
 */
bool SerialInterface::isAttitudeReady() {
	return (fQuatCount==4);
}

Quaternion SerialInterface::getQuaternion() {
	return fData.getQuaternion();
}

VectorFloat SerialInterface::getOmega() {
	return fData.getAngularRate();
}

VectorFloat SerialInterface::getAcceleration() {
	return fData.getLinearAcceleration();
}

VectorFloat SerialInterface::getPosition() {
	return fData.getPosition();
}

VectorFloat SerialInterface::getAlpha() {
	return fData.getAlpha();
}

bool SerialInterface::checkDataAvailable() {
	return isSensorReady();
}

void SerialInterface::disableAll() {
	setMotorPowerAll(0);
}

void SerialInterface::setMotorPowerAll(double power) {
	for(int i=fFirstMotor; i<fLastMotor; ++i){
		cmdPower(i, power);
	}
}

void SerialInterface::setMotorPower(double power, int i) {
	cmdPower(i, power);
}

int SerialInterface::getFirstMotor() {
	return fFirstMotor;
}

int SerialInterface::getLastMotor() {
	return fLastMotor;
}

/**
 * Parse a control command
 * @param s : control command line
 */
void SerialInterface::readCtrlCommand(String s) {
	fCtrlCommandReady = true;
	if(s.startsWith("GOCALIB")) fCtrlCommand = Constants::CtrlCommand::kDOCALIB;
	else fCtrlCommandReady = false;
}

bool SerialInterface::isCtrlCommandReady() {
	return fCtrlCommandReady;
}

Constants::CtrlCommand::ECtrlCommand SerialInterface::getCtrlCommand() {
	fCtrlCommandReady = false;
	return fCtrlCommand;
}

/**
 * Return the received sensor buffer.
 * @return sensor buffer
 */
float* SerialInterface::getBuffer() {
	fBufferCount = 0;
	for(int i=0; i<10; i++){
		Serial.print(fBuffer[i]);
		Serial.print(" ");
	}
	Serial.println("");
	return fBuffer;
}

