/*
 * MainLoop.h
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#ifndef MAINLOOP_H_
#define MAINLOOP_H_
#include "AccGyro/AccGyro.h"
#include "Control/AttitudeLoop.h"
#include "Control/Calibrator.h"
#include "FSM/CalibrationLoop.h"
#include "Communication/SerialInterface.h"
#include "Constants.h"
#include "FSM/StillLoop.h"
#include "Generic/FSMLoop.h"

class MainLoop : public FSMLoop{
public:
	MainLoop();
	virtual ~MainLoop();

	bool processLoop();
	void start();

private:
	void idleLoop();
	void initializationLoop();
	void calibrationLoop();
	void flightLoop();
	void stillLoop();

	void moveToIdle();
	void moveToCalibration();
	void moveToStill();
	void moveToFlight();

	void useDefaultCalib();

	/*void setCalibration();
	void setCalibrationSerial();
	void setCalibrationLocal();
	void calibrateSerial();
	void calibrateSensor();
	void calibrate();*/

	enum FSMState {kINITIALIZING, kIDLE, kCALIBRATING, kSTILL, kFLYING};

	CalibrationLoop fLoopCalib;
	StillLoop fLoopStill;

	bool fSimulate;

	FSMState fState;
	Constants::CtrlCommand::ECtrlCommand fCtrlCommand;

};

#endif /* MAINLOOP_H_ */
