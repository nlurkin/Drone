/*
 * GenericSensor.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef GENERICSENSOR_H_
#define GENERICSENSOR_H_
#include "mathClasses.h"
#include "SerialInterface.h"

class GenericSensor {
public:
	GenericSensor();
	virtual ~GenericSensor();

	Quaternion getQuaternion();
	VectorFloat getOmega();
	VectorFloat getAcceleration();
	VectorFloat getPosition();
	VectorFloat getAlpha();

	//void setSimulate(bool simulate) {Serial.print(F("Setting simulation mode:")); Serial.println(simulate); fSimulate = simulate;}
	void setSerialInterface(SerialInterface *s);
};

#endif /* GENERICSENSOR_H_ */
