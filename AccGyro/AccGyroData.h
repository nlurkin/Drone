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

	VectorInt16 getRawAcceleration();
	VectorFloat getLinearAcceleration();
	VectorFloat getTrueAcceleration();
	VectorFloat getGravity();

	VectorInt16 getRawAngularRate();
	VectorFloat getAngularRate();

	VectorFloat getAlpha();

	VectorFloat getPosition();

	Quaternion getQuaternion();
	uint8_t *getTeaPotPacket();



private:
	void computeAlpha(int timestamp, VectorFloat oldGyroscope);

	int fTimestamp;
	Quaternion fQuaternion;
	VectorInt16 fAcceleration;
	VectorInt16 fGyroscope;
	VectorFloat fAlpha;
	//int16_t fTemperature;
	uint8_t teapotPacket[14];

	float fFullScaleGyroscope;
	float fFullScaleAccelerometer;
};

#endif /* ACCGYRODATA_H_ */
