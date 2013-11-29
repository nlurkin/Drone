/*
 * AttitudeLoop.h
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#ifndef ATTITUDELOOP_H_
#define ATTITUDELOOP_H_

#include <Arduino.h>
#include "helper_3dmath.h"

class AttitudeLoop {
public:
	AttitudeLoop();
	virtual ~AttitudeLoop();

	VectorFloat Compute(Quaternion qM, VectorFloat omegaM);

	const Quaternion& getQRef() const {return fQRef;}
	float getPq() const {return fPQ;}
	float getPOmega() const {return fPOmega;}
	void setQRef(const Quaternion& qRef) {fQRef = qRef;}
	void setP(float pQ, float pOmega) {
		fPQ = pQ;
		fPOmega = pOmega;
	}

private:
	Quaternion fQRef;
	float fPQ, fPOmega;
	VectorFloat fTorque;
};

#endif /* ATTITUDELOOP_H_ */
