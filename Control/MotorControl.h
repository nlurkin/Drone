/*
 * MotorControl.h
 *
 *  Created on: 23 Feb 2014
 *      Author: Nicoas
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

class MotorControl {
public:
	MotorControl();
	virtual ~MotorControl();

	void disableAll();
	void setMotorPowerAll(double power);
	void setMotorPower(double power, int i);

	int getFirstMotor();
	int getLastMotor();
};

#endif /* MOTORCONTROL_H_ */
