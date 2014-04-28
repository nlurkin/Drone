/*
 * AccGyroData.h
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#ifndef ACCGYRODATA_H_
#define ACCGYRODATA_H_

#include <Arduino.h>
#include "mathClasses.h"

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

class AccGyroData {
public:
	AccGyroData();
	virtual ~AccGyroData();

	void setFromBuffer(uint8_t *buffer, int timestamp);
	void setFromSerial(float buffer[10], int timestamp);
	void setFullScaleAccelerometer(uint8_t r);
	void setFullScaleGyroscope(uint8_t r);

	VectorInt<int32_t> getRawAcceleration();
	VectorFloat getLinearAcceleration();
	VectorFloat getTrueAcceleration();
	VectorFloat getGravity();

	VectorInt<int32_t> getRawAngularRate();
	VectorFloat getAngularRate();

	VectorFloat getAlpha();

	VectorFloat getPosition();
	VectorFloat getVelocity();

	Quaternion getQuaternion();
	uint8_t *getTeaPotPacket();



private:
	void computeAlpha(int timestamp, VectorFloat oldGyroscope);
	void computePosition(int timestamp);
	void computeSpeed(int timestamp);

	int fTimestamp;
	Quaternion fQuaternion;
	VectorInt<int32_t> fAcceleration;
	VectorInt<int32_t> fGyroscope;
	VectorFloat fAlpha;
	//int16_t fTemperature;
	uint8_t teapotPacket[14];
	VectorFloat fVelocity;
	VectorFloat fPosition;

	float fFullScaleGyroscope;
	float fFullScaleAccelerometer;

	Kalman<1, 2, 0> filter;
	VectorFloat fOmega;
};

#endif /* ACCGYRODATA_H_ */
