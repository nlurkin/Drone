/*
 * AccGyroData.cpp
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#include "AccGyroData.h"

AccGyroData::AccGyroData(){
	fTemperature = 0;
	teapotPacket[0] = '$';
	teapotPacket[1] = 0x02;
	teapotPacket[2] = 0;
	teapotPacket[3] = 0;
	teapotPacket[4] = 0;
	teapotPacket[5] = 0;
	teapotPacket[6] = 0;
	teapotPacket[7] = 0;
	teapotPacket[8] = 0;
	teapotPacket[9] = 0;
	teapotPacket[10] = 0x00;
	teapotPacket[11] = 0x00;
	teapotPacket[12] = '\r';
	teapotPacket[13] = '\n';

	setFullScaleAccelerometer(0);
	setFullScaleGyroscope(0);
}

AccGyroData::~AccGyroData() {

}


void AccGyroData::setFromBuffer(uint8_t *buffer, int timestamp){
	VectorFloat oldGyro = getTrueGyroscope();

	fQuaternion.w = (buffer[0] << 8) + buffer[1];
	fQuaternion.x = (buffer[4] << 8) + buffer[5];
	fQuaternion.y = (buffer[8] << 8) + buffer[9];
	fQuaternion.z = (buffer[12] << 8) + buffer[13];

	fGyroscope.x = (buffer[16] << 8) + buffer[17];
	fGyroscope.y = (buffer[20] << 8) + buffer[21];
	fGyroscope.z = (buffer[24] << 8) + buffer[25];

	fAcceleration.x = (buffer[28] << 8) + buffer[29];
	fAcceleration.y = (buffer[32] << 8) + buffer[33];
	fAcceleration.z = (buffer[36] << 8) + buffer[37];

	teapotPacket[2] = buffer[0];
	teapotPacket[3] = buffer[1];
	teapotPacket[4] = buffer[4];
	teapotPacket[5] = buffer[5];
	teapotPacket[6] = buffer[8];
	teapotPacket[7] = buffer[9];
	teapotPacket[8] = buffer[12];
	teapotPacket[9] = buffer[13];

	computeAlpha(timestamp, oldGyro);
}

void AccGyroData::setFullScaleAccelerometer(uint8_t r){
	if(r==0) fFullScaleAccelerometer = 8192.0 * 2;
	if(r==1) fFullScaleAccelerometer = 4096.0 * 2;
	if(r==2) fFullScaleAccelerometer = 2048.0 * 2;
	if(r==3) fFullScaleAccelerometer = 1024.0 * 2;
}

void AccGyroData::setFullScaleGyroscope(uint8_t r){
	if(r==0) fFullScaleGyroscope = 131.0 * 2;
	if(r==1) fFullScaleAccelerometer = 65.5 * 2;
	if(r==2) fFullScaleAccelerometer = 32.8 * 2;
	if(r==3) fFullScaleAccelerometer = 16.4 * 2;
}


VectorInt16 AccGyroData::getRawAcceleration(){
	return fAcceleration;
}

VectorFloat AccGyroData::getLinearAcceleration(){
	VectorFloat r;

	r.x = fAcceleration.x/fFullScaleAccelerometer;
	r.y = fAcceleration.y/fFullScaleAccelerometer;
	r.z = fAcceleration.z/fFullScaleAccelerometer;

	return r;
}

VectorFloat AccGyroData::getGravity(){
	VectorFloat r(0., 0., 1.);

	r.rotate(&fQuaternion);

	r = r*fFullScaleAccelerometer;
	return r;
}

VectorFloat AccGyroData::getTrueAcceleration(){
	VectorFloat r;
	r = getLinearAcceleration();
	r = r - getGravity();
	return r;
}

Quaternion AccGyroData::getQuaternion(){
	return fQuaternion;
}

void AccGyroData::setFromSerial(float buffer[10], int timestamp) {

	VectorFloat oldGyro = getTrueGyroscope();

	Serial.print("Reading from buffer: (");
	for(int i =0; i<10; i++){
		Serial.print(buffer[i]);
		Serial.print(",");
	}
	Serial.println(")");
	fQuaternion.w = buffer[0];
	fQuaternion.x = buffer[1];
	fQuaternion.y = buffer[2];
	fQuaternion.z = buffer[3];
	fGyroscope.x = buffer[4];
	fGyroscope.y = buffer[5];
	fGyroscope.z = buffer[6];
	fAcceleration.x = buffer[7];
	fAcceleration.y = buffer[8];
	fAcceleration.z = buffer[9];

	computeAlpha(timestamp, oldGyro);
}

VectorInt16 AccGyroData::getRawGyroscope() {
	return fGyroscope;
}

VectorFloat AccGyroData::getTrueGyroscope() {
	VectorFloat r;

	r.x = fGyroscope.x/fFullScaleGyroscope;
	r.y = fGyroscope.y/fFullScaleGyroscope;
	r.z = fGyroscope.z/fFullScaleGyroscope;

	return r;
}

VectorFloat AccGyroData::getAlpha() {
	return fAlpha;
}

uint8_t *AccGyroData::getTeaPotPacket(){
	teapotPacket[11]++;
	return teapotPacket;
}

void AccGyroData::computeAlpha(int timestamp, VectorFloat oldGyroscope) {
	fAlpha = (getTrueGyroscope() - oldGyroscope)/(timestamp-fTimestamp);
	fTimestamp = timestamp;
}
