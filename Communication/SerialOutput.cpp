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
SerialOutput& SerialOutput::operator <<(const long int s) {
	Serial.print(s);
	return *this;
}

SerialOutput& SerialOutput::operator <<(const double s) {
	Serial.print(s);
	return *this;
}

SerialOutput& endl(SerialOutput& c) {
	Serial.println();
	//delay(100);
	return c;
}

SerialOutput& SerialOutput::operator<<( SerialOutput& (*f)(SerialOutput&) )
  {
      return f(*this);
  }
