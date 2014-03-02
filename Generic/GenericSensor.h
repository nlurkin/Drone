/*
 * GenericSensor.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef GENERICSENSOR_H_
#define GENERICSENSOR_H_

class GenericSensor {
public:
	GenericSensor();
	virtual ~GenericSensor();

	Quaternion getQuaternion();
	VectorFloat getOmega();
	VectorFloat getAcceleration();
	VectorFloat getPosition();
	VectorFloat getAlpha();
};

#endif /* GENERICSENSOR_H_ */
