/*
 * CalibrationLoop.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef CALIBRATIONLOOP_H_
#define CALIBRATIONLOOP_H_

#include "Calibrator.h"
#include "Constants.h"
#include "SimpleControl.h"
#include <EEPROM.h>

class CalibrationLoop {
public:
	enum Path {kFILE, kPROCEDURE};
	enum CalibrationState {kIDLE, kWAITING, kSCANNING, kTAKEOFF, kSTABILIZING, kIDISTURBED, kIMEASUREP, kIMEASUREM,
		kMDISTURBED, kMMEASURES, kMMEASURED, kMBALANCED, kMBALANCES,
		kAPPLY, kLOAD};

	CalibrationLoop();
	virtual ~CalibrationLoop();

	void setCalibPath(Path p);
	bool processLoop();
	void start();

private:
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

	template <class T>
	int EEPROM_writeAnything(int ee, const T& value)
	{
	    const byte* p = (const byte*)(const void*)&value;
	    unsigned int i;
	    for (i = 0; i < sizeof(value); i++)
	        EEPROM.write(ee++, *p++);
	    return i;
	};

	template <class T>
	int EEPROM_readAnything(int ee, T& value)
	{
	    byte* p = (byte*)(void*)&value;
	    unsigned int i;
	    for (i = 0; i < sizeof(value); i++)
	        *p++ = EEPROM.read(ee++);
	    return i;
	};

	double getSqrtMotorPower(int factor);

	Path fPath;
	CalibrationState fState;

	double fCurrentPower;

	Calibrator fCalibrator;

	double calibHeight;

	unsigned long fStopTime;
	double fIPInterval;
	unsigned long fITInterval;
	CalibrationState fNextState;
	int fLoopIndex;
	int fMaxLoop;

	int fCurrentMotor;
	double fMPInterval;
	unsigned long fMTInterval;

	int eepromAddress[19];
};

#endif /* CALIBRATIONLOOP_H_ */

