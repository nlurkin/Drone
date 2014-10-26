/*
 * SimpleControl.cpp
 *
 *  Created on: 23 Feb 2014
 *      Author: Nicoas
 */

#include "Control/SimpleControl.h"
#include <arduino/Arduino.h>
#include "Constants.h"

SimpleControl::SimpleControl() {
	// TODO Auto-generated constructor stub
	fStable = false;
	fKP = 5;
	fKD = 3;

	fZRef = 50;
	fMotorLim = 1;

}

SimpleControl::~SimpleControl() {
	// TODO Auto-generated destructor stub
}

double SimpleControl::loop(double z, double vz) {
	double err = fZRef-z;

	double control = fKP*err - fKD*vz;

	if(fabs(vz)<0.1) fStable=true;

	control=fMotorLim + control/fMotorLim;

	return control;
}

double SimpleControl::getMotorLim() const {
	return fMotorLim;
}

void SimpleControl::setMotorLim(double motorLim) {
	fMotorLim = motorLim;
}

double SimpleControl::getZRef() const {
	return fZRef;
}

void SimpleControl::setZRef(double zRef) {
	fZRef = zRef;
}

bool SimpleControl::isStable() {
	return fStable;
}

void SimpleControl::setKs(double kp, double kd) {
	fKP = kp;
	fKD = kd;
}
