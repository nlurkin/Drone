/*
 * AttitudeLoop.cpp
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#include "AttitudeLoop.h"

AttitudeLoop::AttitudeLoop() {
	// TODO Auto-generated constructor stub

}

AttitudeLoop::~AttitudeLoop() {
	// TODO Auto-generated destructor stub
}

VectorFloat AttitudeLoop::Compute(Quaternion qM, VectorFloat omegaM) {
	Quaternion qErr;
	VectorFloat axisErr;

	qErr = fQRef * qM;

	if(qErr.w < 0) axisErr = -VectorFloat(qErr);
	else axisErr = VectorFloat(qErr);

	fTorque = -axisErr*fPQ - omegaM*fPOmega;
	return fTorque;
}
