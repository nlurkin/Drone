/*
 * SerialOutput.cpp
 *
 *  Created on: 8 Mar 2014
 *      Author: Nicoas
 */

#include "SerialOutput.h"
#include <Arduino.h>

SerialOutput::SerialOutput() {
	// TODO Auto-generated constructor stub

}

SerialOutput::~SerialOutput() {
	// TODO Auto-generated destructor stub
}


SerialOutput& SerialOutput::operator <<(const char* s) {
	Serial.print(s);
	return *this;
}

SerialOutput& SerialOutput::operator <<(const String s) {
	Serial.print(s);
	return *this;
}

SerialOutput& SerialOutput::operator <<(const int s) {
	Serial.print(s);
	return *this;
}

SerialOutput& SerialOutput::operator <<(const cst c) {
	if(c==endl){
		Serial.println();
		//delay(100);
	}
	return *this;
}

//void SerialOutput::print(const String constString) {
//}
