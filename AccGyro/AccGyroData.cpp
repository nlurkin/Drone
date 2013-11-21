/*
 * AccGyroData.cpp
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#include "AccGyroData.h"

AccGyroData::AccGyroData(){
	cartesianAcceleration(0,0,0);
	cartesianGyroscope(0,0,0);
	temperature = 0;
}

AccGyroData::~AccGyroData() {

}

void AccGyroData::setAccelerometerValues(uint16_t x, uint16_t y, uint16_t z){
	cartesianAcceleration(x,y,z);
}

void AccGyroData::setAccelerometerValueX(uint16_t v){
	cartesianAcceleration.x = v;
}

void AccGyroData::setAccelerometerValueY(uint16_t v){
	cartesianAcceleration.y = v;
}

void AccGyroData::setAccelerometerValueZ(uint16_t v){
	cartesianAcceleration.z = v;
}

void AccGyroData::setGyroscopeValues(uint16_t x, uint16_t y, uint16_t z){
	cartesianGyroscope(x,y,z);
}

void AccGyroData::setGyroscopeValueX(uint16_t v){
	cartesianGyroscope.x = v;
}

void AccGyroData::setGyroscopeValueY(uint16_t v){
	cartesianGyroscope.y = v;
}

void AccGyroData::setGyroscopeValueZ(uint16_t v){
	cartesianGyroscope.z = v;
}

void AccGyroData::setTemperatureValue(uint16_t v){
	temperature = v;
}


bool AccGyroData::setValuesFromBuffer7(uint8_t *buffer){
	x_accel = (((int16_t)buffer[0]) << 8) | buffer[1];
	y_accel = (((int16_t)buffer[2]) << 8) | buffer[3];
	z_accel = (((int16_t)buffer[4]) << 8) | buffer[5];
	temperature = (((int16_t)buffer[6]) << 8) | buffer[7];
	x_gyro = (((int16_t)buffer[8]) << 8) | buffer[9];
	y_gyro = (((int16_t)buffer[10]) << 8) | buffer[11];
	z_gyro = (((int16_t)buffer[12]) << 8) | buffer[13];

	if(isAllZero()) return false;
	return true;
}

bool AccGyroData::setValuesFromBufferAcc(uint8_t *buffer){
	x_accel = (((int16_t)buffer[0]) << 8) | buffer[1];
	y_accel = (((int16_t)buffer[2]) << 8) | buffer[3];
	z_accel = (((int16_t)buffer[4]) << 8) | buffer[5];
	if(isAllZero()) return false;
	return true;
}

bool AccGyroData::setValuesFromBufferGyro(uint8_t *buffer){
	x_gyro = (((int16_t)buffer[8]) << 8) | buffer[9];
	y_gyro = (((int16_t)buffer[10]) << 8) | buffer[11];
	z_gyro = (((int16_t)buffer[12]) << 8) | buffer[13];
	if(isAllZero()) return false;
	return true;
}

bool AccGyroData::setValuesFromBuffer6(uint8_t *buffer){
	x_accel = (((int16_t)buffer[0]) << 8) | buffer[1];
	y_accel = (((int16_t)buffer[2]) << 8) | buffer[3];
	z_accel = (((int16_t)buffer[4]) << 8) | buffer[5];
	x_gyro = (((int16_t)buffer[8]) << 8) | buffer[9];
	y_gyro = (((int16_t)buffer[10]) << 8) | buffer[11];
	z_gyro = (((int16_t)buffer[12]) << 8) | buffer[13];
	if(isAllZero()) return false;
	return true;
}


uint16_t AccGyroData::getAccelerometerValueX(){
	return x_accel;
}
uint16_t AccGyroData::getAccelerometerValueY(){
	return y_accel;
}
uint16_t AccGyroData::getAccelerometerValueZ(){
	return z_accel;
}

uint16_t AccGyroData::getGyroscopeValueX(){
	return x_gyro;
}
uint16_t AccGyroData::getGyroscopeValueY(){
	return y_gyro;
}
uint16_t AccGyroData::getGyroscopeValueZ(){
	return z_gyro;
}

double AccGyroData::getTemperatureValue(){
	return ((double)temperature + 12412.0) / 340.0;
}

bool AccGyroData::isAllZero(){
	return (x_accel==0) & (y_accel==0) & (z_accel==0) & (x_gyro==0) & (y_gyro==0) & (z_gyro==0) & (temperature==0);
}
