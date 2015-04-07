/*
 * Constants.h
 *
 *  Created on: 8 Mar 2014
 *      Author: Nicoas
 */

#include "Communication/SerialOutput.h"
class GenericSensor;
class GenericMotor;
class GenericControl;
class SimpleControl;
class AttitudeLoop;

GenericSensor *sSensor;
GenericMotor *sMotor;
GenericControl *sControl;
SimpleControl *sAltitude;
AttitudeLoop *sAttitude;
SerialOutput cout;
bool bigDebug = false;

void PRINTOUT(char const* name){
	if(bigDebug) cout << name << endl;
}
