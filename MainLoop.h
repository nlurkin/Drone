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

class MainLoop {
public:
	MainLoop();
	virtual ~MainLoop();

	void setup();
	void loop();

	void initLoop();

	void setCalibration();
	void setCalibrationSerial();
	void setCalibrationLocal();
	void calibrateSerial();
	void calibrateSensor();
	void calibrate();

private:
	AccGyro sensor;
	AttitudeLoop ctl;
	SerialInterface ser;
	bool fSimulate;
	bool fCalibrated;
	bool fInitialized;
	bool fCalibrationRequested;
};

#endif /* MAINLOOP_H_ */
