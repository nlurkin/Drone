/*
 * MainLoop.h
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#ifndef MAINLOOP_H_
#define MAINLOOP_H_
#include "AccGyro/AccGyro.h"
#include "AttitudeLoop.h"
#include "Control/Calibrator.h"
#include "FSM/CalibrationLoop.h"
#include "Communication/SerialInterface.h"
#include "Constants.h"

class MainLoop {
public:
	MainLoop();
	virtual ~MainLoop();

	void setup();
	void loop();

	void idleLoop();
	void initializationLoop();
	void calibrationLoop();
	void flightLoop();

	void moveToIdle();
	void moveToCalibration();
	void moveToFlight();

	/*void setCalibration();
	void setCalibrationSerial();
	void setCalibrationLocal();
	void calibrateSerial();
	void calibrateSensor();
	void calibrate();*/

private:
	enum MainStates {kINITIALIZING, kIDLE, kCALIBRATING, kFLYING};

	CalibrationLoop fLoopCalib;

	bool fSimulate;

	MainStates fState;
	Constants::CtrlCommand::ECtrlCommand fCtrlCommand;

};

#endif /* MAINLOOP_H_ */
