/*
 * MainLoop.h
 *
 *  Created on: 8 déc. 2013
 *      Author: Nicolas
 */

#ifndef MAINLOOP_H_
#define MAINLOOP_H_
#include "AccGyro.h"
#include "AttitudeLoop.h"
#include "Calibrator.h"
#include "CalibrationLoop.h"
#include "SerialInterface.h"
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


	GenericSensor *fSensor;
	MotorControl *fMotorCtrl;
	//AttitudeLoop ctl;
	//SimpleControl sCtl;
	SerialInterface *fSerial;

	//Calibrator calib;

	bool fSimulate;

	//bool fCalibrated;
	//bool fInitialized;
	//bool fCalibrationRequested;

	MainStates fState;
	//MainStates fNextState;
	Constants::CtrlCommand::ECtrlCommand fCtrlCommand;

};

#endif /* MAINLOOP_H_ */
