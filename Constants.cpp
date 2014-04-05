/*
 * Constants.h
 *
 *  Created on: 8 Mar 2014
 *      Author: Nicoas
 */

#include "SerialOutput.h"
class GenericSensor;
class GenericMotor;
class GenericControl;
class SimpleControl;

GenericSensor *sSensor;
GenericMotor *sMotor;
GenericControl *sControl;
SimpleControl *sAltitude;
SerialOutput cout;
bool bigDebug = false;

void PRINTOUT(char const* name){
	if(bigDebug) cout << name << endl;
}
