/*
 * AttitudeLoop.cpp
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#include "Control/AttitudeLoop.h"

AttitudeLoop::AttitudeLoop() {
	fPOmega = 0;
	fPQ = 0;
}

AttitudeLoop::~AttitudeLoop() {
}

VectorFloat AttitudeLoop::ComputePP(Quaternion qM, VectorFloat omegaM) {
	Quaternion qErr;
	VectorFloat axisErr;

	Serial.print("Printing qRef ");
	fQRef.print();
	Serial.print("Printing qM ");
	qM.print();
	qErr = fQRef.conjugate() * qM;
	Serial.print("Printing qErr ");
	qErr.print();

	if(qErr.w < 0) axisErr = VectorFloat(qErr);
	else axisErr = -VectorFloat(qErr);

	Serial.print("Printing axisErr ");
	axisErr.print();
	Serial.print("Printing omegaM ");
	omegaM.print();
	Serial.print("Printing torque without I ");
	(axisErr*fPQ - omegaM*fPOmega).print();
	fTorque = fI * (axisErr*fPQ - omegaM*fPOmega);
	Serial.print("Printing fTorque ");
	fTorque.print();


	return fTorque;
}

motorFactor_t AttitudeLoop::Compute(Quaternion qM, VectorFloat omegaM) {
	motorFactor_t F;

	//Compute necessary torque
	ComputePP(qM, omegaM);

	//Fit the desired torque vector with the motor response function to extract the 4 motor factors
	return F;
}

void AttitudeLoop::setM(int motor, double Rx, double Ry, double Rz, double RT) {
	motorConstants_t *m;
	if(motor==0) m = &fM1;
	else if(motor==1) m = &fM2;
	else if(motor==2) m = &fM3;
	else if(motor==3) m = &fM4;
	else return;

	m->Rx = Rx;
	m->Ry = Ry;
	m->Rz = Rz;
	m->RT = RT;
}

void AttitudeLoop::printI(){
	Serial.print("Ixx=");
	Serial.print(fI(0,0));
	Serial.print(", Iyy=");
	Serial.print(fI(1,1));
	Serial.print(", Izz=");
	Serial.println(fI(2,2));
}
