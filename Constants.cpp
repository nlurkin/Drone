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


GenericSensor *sSensor;
GenericMotor *sMotor;
GenericControl *sControl;
SerialOutput cout;
bool bigDebug = false;

void PRINTOUT(char const* name){
	if(bigDebug) cout << name << SerialOutput::endl;
}
