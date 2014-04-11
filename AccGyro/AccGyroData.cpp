/*
 * AccGyroData.cpp
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#include "AccGyroData.h"
#include "Constants.h"

/**
 * Constructor.
 * Initialize the teapotPacket array (explain).
 * Set the default scale of gyroscope and accelerometer.
 */
AccGyroData::AccGyroData(){
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

/**
 * Destructor.
 */
AccGyroData::~AccGyroData() {

}

/**
 * Parse the buffer to fill the internal structures
 * @param buffer: input buffer (explain structure)
 * @param timestamp: timestamp of the data for integration and derivative
 */
void AccGyroData::setFromBuffer(uint8_t *buffer, int timestamp){
	VectorFloat oldGyro = getAngularRate();

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

/**
 * Change the scale of the accelerometer.
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * @param r: Scale (0 to 3)
 */
void AccGyroData::setFullScaleAccelerometer(uint8_t r){
	if(r==0) fFullScaleAccelerometer = 8192.0 * 2;
	if(r==1) fFullScaleAccelerometer = 4096.0 * 2;
	if(r==2) fFullScaleAccelerometer = 2048.0 * 2;
	if(r==3) fFullScaleAccelerometer = 1024.0 * 2;
}

/**
 * Change the scale of the gyroscope.
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * @param r: Scale (0 to 3)
 */
void AccGyroData::setFullScaleGyroscope(uint8_t r){
	if(r==0) fFullScaleGyroscope = 131.0 * 2;
	if(r==1) fFullScaleAccelerometer = 65.5 * 2;
	if(r==2) fFullScaleAccelerometer = 32.8 * 2;
	if(r==3) fFullScaleAccelerometer = 16.4 * 2;
}

/**
 * Return the raw acceleration vector (as measured by the sensor)
 * @return VectorInt16
 */
VectorInt<int32_t> AccGyroData::getRawAcceleration(){
	return fAcceleration;
}

/**
 * Return the linear acceleration vector (accelerometer full scale applied)
 * @return VectorFloat
 */
VectorFloat AccGyroData::getLinearAcceleration(){
	VectorFloat r;

	r.x = fAcceleration.x/fFullScaleAccelerometer;
	r.y = fAcceleration.y/fFullScaleAccelerometer;
	r.z = fAcceleration.z/fFullScaleAccelerometer;

	return r;
}

/**
 * Return the gravity vector (in the sensor frame)
 * @return VectorFloat
 */
VectorFloat AccGyroData::getGravity(){
	VectorFloat r(0., 0., 1.);

	r.rotate(fQuaternion);

	r = r*fFullScaleAccelerometer;
	return r;
}

/**
 * Return the true acceleration (linear acceleration - gravity)
 * @return VectorFloat
 */
VectorFloat AccGyroData::getTrueAcceleration(){
	VectorFloat r;
	r = getLinearAcceleration();
	r = r - getGravity();
	return r;
}

/**
 * Return the current attitude quaternion.
 * @return Quaternion
 */
Quaternion AccGyroData::getQuaternion(){
	return fQuaternion;
}

/**
 * Parse the serial interface buffer to fill the internal structure.
 * @param buffer : Serial float buffer (0-4:quaternion; 5-6:gyroscope; 7-9:accelerometer)
 * @param timestamp : data timestamp for integration and derivation
 */
void AccGyroData::setFromSerial(float buffer[10], int timestamp) {

	VectorFloat oldGyro = getAngularRate();

	cout << F("Reading from buffer: (");
	for(int i =0; i<10; i++){
		cout << buffer[i] << ",";
	}
	cout << ")" << endl;
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
	computeSpeed(timestamp);
	computePosition(timestamp);
	fTimestamp = timestamp;
}

/**
 * Return the angular acceleration.
 * @return VectorFloat
 */
VectorFloat AccGyroData::getAlpha() {
	return fAlpha;
}

/**
 * Return the teapotPacket structure
 * @return address to uint8_t[13]
 */
uint8_t *AccGyroData::getTeaPotPacket(){
	teapotPacket[11]++;
	return teapotPacket;
}

/**
 * Return the current (integrated) position (relative the the initial body position).
 * @return VectorFloat
 */
VectorFloat AccGyroData::getPosition() {
	return fPosition;
}

/**
 * Compute the angular acceleration (by derivative of angular velocity)
 * @param timestamp : current timestamp
 * @param oldGyroscope : previous angular velocities.
 */
void AccGyroData::computeAlpha(int timestamp, VectorFloat oldGyroscope) {
	fAlpha = (getAngularRate() - oldGyroscope)/(timestamp-fTimestamp);
}

/**
 * Return the raw angular velocity (as measured by the gyroscope)
 * @return VectorInt16
 */
VectorInt<int32_t> AccGyroData::getRawAngularRate() {
	return fGyroscope;
}

/**
 * Return the angular velocity (gyroscope full scale applied).
 * @return VectorFloat
 */
VectorFloat AccGyroData::getAngularRate() {
	VectorFloat r;

	r.x = fGyroscope.x/fFullScaleGyroscope;
	r.y = fGyroscope.y/fFullScaleGyroscope;
	r.z = fGyroscope.z/fFullScaleGyroscope;

	return r;
}

void AccGyroData::computePosition(int timestamp) {
	float dt = (timestamp - fTimestamp)/1000.;
	fPosition = fPosition + fVelocity*dt;
}

VectorFloat AccGyroData::getVelocity() {
	return fVelocity;
}

void AccGyroData::computeSpeed(int timestamp) {
	float dt = (timestamp - fTimestamp)/1000.;
	//cout << "dt " << dt << SerialOutput::endl;
	fVelocity = fVelocity + getLinearAcceleration()*dt;
	//cout << "Velocity " << fVelocity.z  << SerialOutput::endl;
}
