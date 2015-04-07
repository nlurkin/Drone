/*
 * FlightLoop.cpp
 *
 *  Created on: 7 Apr 2015
 *      Author: Nicoas
 */

#include "FSM/FlightLoop.h"

FlightLoop::FlightLoop() {
	fState = kIDLE;
}

FlightLoop::~FlightLoop() {
}

bool FlightLoop::processLoop() {
	PRINTOUT("processLoop");
	bool ret = false;
	switch(fState){
	case kIDLE:
		ret = true;
		break;
	default:
		//Should not happen
		break;
	}

	return ret;
}

void FlightLoop::start() {
	PRINTOUT("start");
	//fState = kSTABILIZING;
	//sAttitude->setQRef(Quaternion());
}
