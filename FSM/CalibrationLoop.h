/*
 * CalibrationLoop.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef CALIBRATIONLOOP_H_
#define CALIBRATIONLOOP_H_

#include "Calibrator.h"
class GenericSensor;
class MotorControl;

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

	Path fPath;
	CalibrationState fState;

	int fCurrentPower;

	GenericSensor *fSensor;
	MotorControl *fMotorControl;
	Calibrator fCalibrator;

	double calibHeight;

	unsigned long fStopTime;
	int fIPInterval;
	unsigned long fITInterval;
	CalibrationState fNextState;
	int fLoopIndex;
	int fMaxLoop;

	int fCurrentMotor;
	int fMPInterval;
	unsigned long fMTInterval;

};

#endif /* CALIBRATIONLOOP_H_ */

