/*
 * GenericSensor.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef GENERICSENSOR_H_
#define GENERICSENSOR_H_
#include "Math/mathClasses.h"

class GenericSensor {
public:
	GenericSensor(){};
	virtual ~GenericSensor(){};

	virtual Quaternion getQuaternion()=0;
	virtual VectorFloat getOmega()=0;
	virtual VectorFloat getAcceleration()=0;
	virtual VectorFloat getPosition()=0;
	virtual VectorFloat getAlpha()=0;
	virtual VectorFloat getVelocity()=0;

	virtual unsigned int getTime()=0;

	//virtual void setSerialInterface(SerialInterface *s)=0;

	virtual bool checkDataAvailable()=0;
};

#endif /* GENERICSENSOR_H_ */
