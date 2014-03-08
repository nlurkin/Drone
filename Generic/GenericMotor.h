/*
 * GenericMotor.h
 *
 *  Created on: 23 Feb 2014
 *      Author: Nicoas
 */

#ifndef GENERICMOTOR_H_
#define GENERICMOTOR_H_

class GenericMotor {
public:
	GenericMotor();
	virtual ~GenericMotor();

	virtual void disableAll()=0;
	virtual void setMotorPowerAll(double power)=0;
	virtual void setMotorPower(double power, int i)=0;

	virtual int getFirstMotor()=0;
	virtual int getLastMotor()=0;
};

#endif /* MOTORCONTROL_H_ */
