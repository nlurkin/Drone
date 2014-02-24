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

	/*void setCalibration();
	void setCalibrationSerial();
	void setCalibrationLocal();
	void calibrateSerial();
	void calibrateSensor();
	void calibrate();*/

private:
	enum States {kIDLE, kINITIALIZED, kCALIBRATING, kFLYING};

	AccGyro sensor;
	AttitudeLoop ctl;
	SimpleControl sCtl;
	SerialInterface ser;
	MotorControl mCtrl;
	Calibrator calib;

	bool fSimulate;
	//bool fCalibrated;
	//bool fInitialized;
	//bool fCalibrationRequested;

	States fState;
};

#endif /* MAINLOOP_H_ */
