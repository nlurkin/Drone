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

class AccGyroData {
public:
	AccGyroData();
	virtual ~AccGyroData();

	void setAccelerometerValues(uint16_t x, uint16_t y, uint16_t z);
	void setAccelerometerValueX(uint16_t v);
	void setAccelerometerValueY(uint16_t v);
	void setAccelerometerValueZ(uint16_t v);

	void setGyroscopeValues(uint16_t w, uint16_t x, uint16_t y, uint16_t z);

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

	bool isAllZero();
private:
	VectorFloat cartesianAcceleration;
	Quaternion quaternioGyroscope;
	int16_t temperature;
};

#endif /* ACCGYRODATA_H_ */
