/*
 * SerialOutput.h
 *
 *  Created on: 8 Mar 2014
 *      Author: Nicoas
 */

#ifndef SERIALOUTPUT_H_
#define SERIALOUTPUT_H_

#include <Arduino.h>

class SerialOutput {
public:
	SerialOutput();
	virtual ~SerialOutput();

	SerialOutput& operator<<(const char* s);
	SerialOutput& operator<<(const String s);
	SerialOutput& operator<<(const int s);
	SerialOutput& operator<<(const long int s);
	SerialOutput& operator<<(const double s);
	//SerialOutput& operator<<(const cst s);
	SerialOutput& operator<<( SerialOutput& (*f)(SerialOutput&) );
};

SerialOutput& endl(SerialOutput& s);

#endif /* SERIALOUTPUT_H_ */
