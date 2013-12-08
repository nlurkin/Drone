/*
 * AttitudeLoop.cpp
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#include "AttitudeLoop.h"

AttitudeLoop::AttitudeLoop() {
	// TODO Auto-generated constructor stub
	fPOmega = 0;
	fPQ = 0;
}

AttitudeLoop::~AttitudeLoop() {
	// TODO Auto-generated destructor stub
}

VectorFloat AttitudeLoop::ComputePP(Quaternion qM, VectorFloat omegaM) {
	Quaternion qErr;
	VectorFloat axisErr;

	qErr = fQRef * qM;

	if(qErr.w < 0) axisErr = -VectorFloat(qErr);
	else axisErr = VectorFloat(qErr);

	fTorque = -axisErr*fPQ - omegaM*fPOmega;
	return fTorque;
}

motorFactor_t AttitudeLoop::Compute(Quaternion qM, VectorFloat omegaM) {
	motorFactor_t F;

	//Compute necessary torque
	ComputePP(qM, omegaM);

	//Fit the desired torque vector with the motor response function to extract the 4 motor factors
	return F;
}

void AttitudeLoop::printI(){
	Serial.print("Ixx=");
	Serial.print(fI(0,0));
	Serial.print(", Iyy=");
	Serial.print(fI(1,1));
	Serial.print(", Izz=");
	Serial.println(fI(2,2));
}