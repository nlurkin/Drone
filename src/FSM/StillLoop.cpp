/*
 * StillLoop.cpp
 *
 *  Created on: 15 Feb 2015
 *      Author: Nicoas
 */

#include "FSM/StillLoop.h"
#include "Constants.h"
#include "Math/Quaternion.h"
#include "Generic/GenericMotor.h"
#include "Generic/GenericSensor.h"
#include "Control/AttitudeLoop.h"

StillLoop::StillLoop() {
	fState = kIDLE;
}

StillLoop::~StillLoop() {

}

bool StillLoop::processLoop(){
	PRINTOUT("processLoop");
	bool ret = false;
	switch(fState){
	case kIDLE:
		ret = true;
		break;
	case kSTABILIZING:
		stabilize();
		break;
	case kSTABILIZED:
		maintain();
		break;
	default:
		//Should not happen
		break;
	}

	return ret;
}


void StillLoop::start(){
	PRINTOUT("start");
	fState = kSTABILIZING;
	sAttitude->setQRef(Quaternion());
}

void StillLoop::stabilize(){
	if(sSensor->checkDataAvailable()){
		motorFactor m = sAttitude->Compute(sSensor->getQuaternion(), sSensor->getOmega());
		sMotor->setMotorPower(m.m1, 0);
		sMotor->setMotorPower(m.m2, 1);
		sMotor->setMotorPower(m.m3, 2);
		sMotor->setMotorPower(m.m4, 3);
	}
}

void StillLoop::maintain(){

}
