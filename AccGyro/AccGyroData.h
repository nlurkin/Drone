/*
 * AccGyroData.h
 *
 *  Created on: 17 août 2013
 *      Author: Nicolas
 */

#ifndef ACCGYRODATA_H_
#define ACCGYRODATA_H_

#include <Arduino.h>
#include "helper_3dmath.h"

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

	void setFromBuffer(uint8_t *buffer);
	void setFullScaleAccelerometer(uint8_t r);
	void setFullScaleGyroscope(uint8_t r);

	VectorInt16 getRawAcceleration();
	VectorFloat getLinearAcceleration();
	VectorFloat getTrueAcceleration();
	VectorFloat getGravity();

	Quaternion getQuaternion();
	uint8_t *getTeaPotPacket();

	VectorInt16 getRawAngularRate();
	VectorFloat getAngularRate();

	/*void setAccelerometerValues(uint16_t x, uint16_t y, uint16_t z);
	void setAccelerometerValueX(uint16_t v);
	void setAccelerometerValueY(uint16_t v);
	void setAccelerometerValueZ(uint16_t v);

	void setTemperatureValue(uint16_t v);

	bool setValuesFromBufferAcc(uint8_t *buffer);
	bool setValuesFromBufferGyro(uint8_t *buffer);
	bool setValuesFromBuffer6(uint8_t *buffer);
	bool setValuesFromBuffer7(uint8_t *buffer);

	uint16_t getAccelerometerValueX();
	uint16_t getAccelerometerValueY();
	uint16_t getAccelerometerValueZ();

	uint16_t getGyroscopeValueX();
	uint16_t getGyroscopeValueY();
	uint16_t getGyroscopeValueZ();

	double getTemperatureValue();

	bool isAllZero();*/
private:
	Quaternion fQuaternion;
	VectorInt16 fAcceleration;
	VectorInt16 fGyroscope;
	int16_t fTemperature;
	uint8_t teapotPacket[14];

	//int16_t temperature;

	float fFullScaleGyroscope;
	float fFullScaleAccelerometer;
};

#endif /* ACCGYRODATA_H_ */
