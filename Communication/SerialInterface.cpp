/*
 * SerialInterface.cpp
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#include "SerialInterface.h"
#include "SerialOutput.h"

SerialInterface::SerialInterface() {
	cout << "Drone serial interface initialized" << SerialOutput::endl;
	fICount = 0;
	fBufferCount = 0;
	fQuatCount = 0;

	fCtrlCommand = Constants::CtrlCommand::kNONE;
	fCtrlCommandReady = false;

	fTime = 0;
	fFirstMotor = 0;
	fLastMotor = 3;

	fSimKCount = 0;
	fSimKP = 0;
	fSimKD = 0;
}

SerialInterface::~SerialInterface() {
}

/**
 * Command sent to change motor power
 * @param motor: index of the motor
 * @param power: new power
 */
void SerialInterface::cmdPower(int motor, double power) {
	PRINTOUT("cmdPower");
	cout << "CMD:power:" << motor << ":" << power << SerialOutput::endl;
}

/**
 * Command to request the simulator to send the I matrix
 */
void SerialInterface::cmdRequestI(){
	PRINTOUT("cmdRequestI");
	cout << "CMD:sendI" << SerialOutput::endl;
}

/**
 * Loop method. Check if something to be read on pipe and parse it.
 * @return false
 */
bool SerialInterface::read(){
	PRINTOUT("read");
	String s;
	bool r = false;

	while(Serial.available()>0){
		s = Serial.readStringUntil('\n');
		//cout << "Receiving from Serial: " << s << SerialOutput::endl;
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
	PRINTOUT("readData");
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
	PRINTOUT("readCmd");
	if(s.startsWith("TRCK:")){				//Setting a new attitude reference
		readNewAttitude(s.substring(5));
	}
	else if(s.startsWith("CTRL:")){				//Control command
		readCtrlCommand(s.substring(5));
	}
	else if(s.startsWith("SETK:")){
		readKValues(s.substring(5));
	}
}

void SerialInterface::readKValues(String s){
	PRINTOUT("readKValues");

	if(fSimKCount==2) fSimKCount=0;
	if(s.startsWith("SIMKP:")) fSimKP = atof(s.substring(6).c_str());
	else if(s.startsWith("SIMKD:")) fSimKD = atof(s.substring(6).c_str());

	fSimKCount++;
}



/**
 * Parse a sensor line. Waiting 10 values.
 * @param s: sensor line
 */
void SerialInterface::readSensor(String s) {
	PRINTOUT("readSensor");
	if(fBufferCount==11) fBufferCount=0;

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
	else if(s.startsWith("TIME:")) fTime = atof(s.substring(5).c_str())*1000;
	fBufferCount++;
	if(fBufferCount==11){
		//cout << "Full buffer received " << fTime << SerialOutput::endl;
		fData.setFromSerial(fBuffer, fTime);
	}
}

/**
 * Parse a I matrix line. Waiting 3 values.
 * @param s: matrix line
 */
void SerialInterface::readIMat(String s) {
	PRINTOUT("readIMat");
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
	PRINTOUT("isIReady");
	return (fICount==3);
}

/**
 *
 * @return I matrix
 */
VectorFloat SerialInterface::getI() {
	PRINTOUT("getI");
	fICount = 0;
	return fI;
}

/**
 * Did we receive a full sensor buffer?
 * @return true if full sensor buffer received.
 */
bool SerialInterface::isSensorReady(){
	PRINTOUT("isSensorReady");
	//TODO faire comme ça ou pas?
	bool isReady = (fBufferCount==11);
	if(isReady) {
		fBufferCount = 0;
	}
	return isReady;
}

/**
 * Sending a command to change the current applied torque.
 * @param tau: torque
 */
void SerialInterface::cmdTorque(VectorFloat tau) {
	PRINTOUT("cmdTorque");
	cout << "CMD:TAUS:TAUX:" << tau.x << SerialOutput::endl;
	cout << "CMD:TAUS:TAUY:" << tau.y << SerialOutput::endl;
	cout << "CMD:TAUS:TAUZ:" << tau.z << SerialOutput::endl;
}

/**
 * Parse the new attitude quaternion line.
 * @param s: attitude quaternion line
 */
void SerialInterface::readNewAttitude(String s) {
	PRINTOUT("readNewAttitude");
	if(fQuatCount==4) fQuatCount=0;

	if(s.startsWith("QUAW:")){
		fRefQuat.w = atof(s.substring(5).c_str());
		fQuatCount=0;
	}
	else if(s.startsWith("QUAX:")) fRefQuat.x = atof(s.substring(5).c_str());
	else if(s.startsWith("QUAY:")) fRefQuat.y = atof(s.substring(5).c_str());
	else if(s.startsWith("QUAZ:")) fRefQuat.z = atof(s.substring(5).c_str());
	fQuatCount++;
	if(fQuatCount==4) cout << "Full attitude quaternion received" << SerialOutput::endl;
}

/**
 * Did we receive a full new attitude quaternion?
 * @return true if full new attitude quaternion received
 */
bool SerialInterface::isAttitudeReady() {
	PRINTOUT("isAttitudeReady");
	return (fQuatCount==4);
}

Quaternion SerialInterface::getQuaternion() {
	PRINTOUT("getQuaternion");
	return fData.getQuaternion();
}

VectorFloat SerialInterface::getOmega() {
	PRINTOUT("getOmega");
	return fData.getAngularRate();
}

VectorFloat SerialInterface::getAcceleration() {
	PRINTOUT("getAcceleration");
	return fData.getLinearAcceleration();
}

VectorFloat SerialInterface::getPosition() {
	PRINTOUT("getPosition");
	return fData.getPosition();
}

VectorFloat SerialInterface::getAlpha() {
	PRINTOUT("getAlpha");
	return fData.getAlpha();
}

bool SerialInterface::checkDataAvailable() {
	PRINTOUT("checkDataAvailable");
	return isSensorReady();
}

void SerialInterface::disableAll() {
	PRINTOUT("disableAll");
	setMotorPowerAll(0);
	cmdNextStep();
}

void SerialInterface::setMotorPowerAll(double power) {
	PRINTOUT("setMotorPowerAll");
	for(int i=fFirstMotor; i<=fLastMotor; ++i){
		cmdPower(i, power);
	}
	cmdNextStep();
}

void SerialInterface::setMotorPower(double power, int i) {
	PRINTOUT("setMotorPower");
	cmdPower(i, power);
	cmdNextStep();
}

int SerialInterface::getFirstMotor() {
	PRINTOUT("getFirstMotor");
	return fFirstMotor;
}

int SerialInterface::getLastMotor() {
	PRINTOUT("getLastMotor");
	return fLastMotor;
}

/**
 * Parse a control command
 * @param s : control command line
 */
void SerialInterface::readCtrlCommand(String s) {
	PRINTOUT("readCtrlCommand");
	cout << "Reading control " << s << SerialOutput::endl;

	fCtrlCommandReady = true;
	if(s.startsWith("GOCALIB")) fCtrlCommand = Constants::CtrlCommand::kDOCALIB;
	else if(s.startsWith("GODEBUG")) fCtrlCommand = Constants::CtrlCommand::kDODEBUG;
	else fCtrlCommandReady = false;
}

bool SerialInterface::isCtrlCommandReady() {
	PRINTOUT("isCtrlCommandReady");
	return fCtrlCommandReady;
}

Constants::CtrlCommand::ECtrlCommand SerialInterface::getCtrlCommand() {
	PRINTOUT("getCtrlCommand");
	fCtrlCommandReady = false;
	return fCtrlCommand;
}

void SerialInterface::cmdNextStep() {
	cout << "CMD:NEXT" << SerialOutput::endl;
}

VectorFloat SerialInterface::getVelocity() {
	return fData.getVelocity();
}

bool SerialInterface::isSimpleKFactorsReady() {
	bool r = (fSimKCount==2);
	if(r) fSimKCount = 0;
	return r;
}

double SerialInterface::getSimpleKP() {
	return fSimKP;
}

double SerialInterface::getSimpleKD() {
	return fSimKD;
}

int SerialInterface::getTime() {
	return fTime;
}

/**
 * Return the received sensor buffer.
 * @return sensor buffer
 */
float* SerialInterface::getBuffer() {
	PRINTOUT("getBuffer");
	fBufferCount = 0;
	for(int i=0; i<10; i++){
		Serial.print(fBuffer[i]);
		Serial.print(" ");
	}
	Serial.println("");
	return fBuffer;
}

