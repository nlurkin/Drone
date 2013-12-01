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


void AccGyroData::setFromBuffer(uint8_t *buffer){
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

void AccGyroData::setFromSerial(float buffer[10]) {

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
}

uint8_t *AccGyroData::getTeaPotPacket(){
	teapotPacket[11]++;
	return teapotPacket;
}

//void AccGyroData::setAccelerometerValues(uint16_t x, uint16_t y, uint16_t z){
	//	cartesianAcceleration.x = x;
//	cartesianAcceleration.y = y;
//	cartesianAcceleration.z = z;
//}
//
//void AccGyroData::setAccelerometerValueX(uint16_t v){
//	cartesianAcceleration.x = v;
//}
//
//void AccGyroData::setAccelerometerValueY(uint16_t v){
//	cartesianAcceleration.y = v;
//}
//
//void AccGyroData::setAccelerometerValueZ(uint16_t v){
//	cartesianAcceleration.z = v;
//}
//
//void AccGyroData::setGyroscopeValues(uint16_t x, uint16_t y, uint16_t z){
//	cartesianGyroscope(x,y,z);
//}
//
//void AccGyroData::setGyroscopeValueX(uint16_t v){
//	cartesianGyroscope.x = v;
//}
//
//void AccGyroData::setGyroscopeValueY(uint16_t v){
//	cartesianGyroscope.y = v;
//}
//
//void AccGyroData::setGyroscopeValueZ(uint16_t v){
//	cartesianGyroscope.z = v;
//}
//
//void AccGyroData::setTemperatureValue(uint16_t v){
//	temperature = v;
//}
//
//
//bool AccGyroData::setValuesFromBuffer7(uint8_t *buffer){
//	x_accel = (((int16_t)buffer[0]) << 8) | buffer[1];
//	y_accel = (((int16_t)buffer[2]) << 8) | buffer[3];
//	z_accel = (((int16_t)buffer[4]) << 8) | buffer[5];
//	temperature = (((int16_t)buffer[6]) << 8) | buffer[7];
//	x_gyro = (((int16_t)buffer[8]) << 8) | buffer[9];
//	y_gyro = (((int16_t)buffer[10]) << 8) | buffer[11];
//	z_gyro = (((int16_t)buffer[12]) << 8) | buffer[13];
//
//	if(isAllZero()) return false;
//	return true;
//}
//
//bool AccGyroData::setValuesFromBufferAcc(uint8_t *buffer){
//	x_accel = (((int16_t)buffer[0]) << 8) | buffer[1];
//	y_accel = (((int16_t)buffer[2]) << 8) | buffer[3];
//	z_accel = (((int16_t)buffer[4]) << 8) | buffer[5];
//	if(isAllZero()) return false;
//	return true;
//}
//
//bool AccGyroData::setValuesFromBufferGyro(uint8_t *buffer){
//	x_gyro = (((int16_t)buffer[8]) << 8) | buffer[9];
//	y_gyro = (((int16_t)buffer[10]) << 8) | buffer[11];
//	z_gyro = (((int16_t)buffer[12]) << 8) | buffer[13];
//	if(isAllZero()) return false;
//	return true;
//}
//
//bool AccGyroData::setValuesFromBuffer6(uint8_t *buffer){
//	x_accel = (((int16_t)buffer[0]) << 8) | buffer[1];
//	y_accel = (((int16_t)buffer[2]) << 8) | buffer[3];
//	z_accel = (((int16_t)buffer[4]) << 8) | buffer[5];
//	x_gyro = (((int16_t)buffer[8]) << 8) | buffer[9];
//	y_gyro = (((int16_t)buffer[10]) << 8) | buffer[11];
//	z_gyro = (((int16_t)buffer[12]) << 8) | buffer[13];
//	if(isAllZero()) return false;
//	return true;
//}
//
//
//uint16_t AccGyroData::getAccelerometerValueX(){
//	return x_accel;
//}
//uint16_t AccGyroData::getAccelerometerValueY(){
//	return y_accel;
//}
//uint16_t AccGyroData::getAccelerometerValueZ(){
//	return z_accel;
//}
//
//uint16_t AccGyroData::getGyroscopeValueX(){
//	return x_gyro;
//}
//uint16_t AccGyroData::getGyroscopeValueY(){
//	return y_gyro;
//}
//uint16_t AccGyroData::getGyroscopeValueZ(){
//	return z_gyro;
//}
//
//double AccGyroData::getTemperatureValue(){
//	return ((double)temperature + 12412.0) / 340.0;
//}
//
//bool AccGyroData::isAllZero(){
//	return (x_accel==0) & (y_accel==0) & (z_accel==0) & (x_gyro==0) & (y_gyro==0) & (z_gyro==0) & (temperature==0);
//}
