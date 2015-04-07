/*
 * CalibrationLoop.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef CALIBRATIONLOOP_H_
#define CALIBRATIONLOOP_H_

#include "Control/Calibrator.h"
#include "Constants.h"
#include "Control/SimpleControl.h"
#include <avr/eeprom.h>
#include "Generic/FSMLoop.h"

class CalibrationLoop : public FSMLoop {
public:
	enum Path {kFILE, kPROCEDURE};

	CalibrationLoop();
	virtual ~CalibrationLoop();

	void setCalibPath(Path p);
	bool processLoop();
	void start();

private:
	enum FSMState {kIDLE, kWAITING, kSCANNING, kTAKEOFF, kSTABILIZING, kIDISTURBED, kIMEASUREP, kIMEASUREM,
		kMDISTURBED, kMMEASURES, kMMEASURED, kMBALANCED, kMBALANCES,
		kAPPLY, kLOAD};

	//Init
	void scanP();
	void takeOff();
	void stabilize();
	void wait();

	//I
	void iDisturbed();
	void measureP();
	void measureM();

	//Motor
	void mDisturbed();
	void measureS();
	void measureD();
	void mBalancedD();
	void mBalancedS();

	//Computing
	void load();
	void compute();
	void apply();

	/*template <class T>
	int EEPROM_writeAnything(int ee, const T& value)
	{
	    const byte* p = (const byte*)(const void*)&value;
	    unsigned int i;
	    for (i = 0; i < sizeof(value); i++)
	        eeprom_write_byte(ee++, *p++);
	    return i;
	};*/

	/*template <class T>
	int EEPROM_readAnything(int ee, T& value)
	{
	    byte* p = (byte*)(void*)&value;
	    unsigned int i;
	    for (i = 0; i < sizeof(value); i++)
	        *p++ = eeprom_read_byte(ee++);
	    return i;
	};*/

	double getSqrtMotorPower(int factor);

	Path fPath;
	FSMState fState;

	double fCurrentPower;

	Calibrator fCalibrator;

	double calibHeight;

	unsigned long fStopTime;
	double fIPInterval;
	unsigned long fITInterval;
	FSMState fNextState;
	int fLoopIndex;
	int fMaxLoop;

	int fCurrentMotor;
	double fMPInterval;
	unsigned long fMTInterval;

	int eepromAddress[19];
};

#endif /* CALIBRATIONLOOP_H_ */

