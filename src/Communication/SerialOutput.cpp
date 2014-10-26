/*
 * SerialOutput.cpp
 *
 *  Created on: 8 Mar 2014
 *      Author: Nicoas
 */

#include "Communication/SerialOutput.h"
#include <arduino/Arduino.h>
//#include "MemoryFree.h"

void SerialOutput::printDouble(double val, byte precision) {
	if(val<0) Serial.print('-');
	Serial.print(int(fabs(val)));                                     // Print int part
	if( precision > 0) {                                         // Print decimal part
		Serial.print(".");
		unsigned long frac, mult = 1;
		byte padding = precision -1;
		while(precision--) mult *=10;
		if(val >= 0) frac = (val - int(val)) * mult; else frac = (int(val) - val) * mult;
		unsigned long frac1 = frac;
		while(frac1 /= 10) padding--;
		while(padding--) Serial.print("0");
		Serial.print(frac,DEC) ;
	}
}

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
	printDouble(s, 5);
	return *this;
}

SerialOutput& SerialOutput::operator<<(const __FlashStringHelper *s){
	Serial.print(s);

	return *this;
}

SerialOutput& endl(SerialOutput& c) {
	Serial.println();
	//Serial.println(freeMemory());
	//delay(100);
	return c;
}

SerialOutput& SerialOutput::operator<<( SerialOutput& (*f)(SerialOutput&) )
{
	return f(*this);
}
