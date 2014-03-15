/*
 * SimpleControl.h
 *
 *  Created on: 23 Feb 2014
 *      Author: Nicoas
 */

#ifndef SIMPLECONTROL_H_
#define SIMPLECONTROL_H_

class SimpleControl {
public:
	SimpleControl();
	virtual ~SimpleControl();

	double loop(double z, double vz);
	double getMotorLim() const;
	void setMotorLim(double motorLim);
	double getZRef() const;
	void setZRef(double zRef);

	bool isStable();

	void setKs(double kp, double kd);
private:
	double fZRef;
	double fMotorLim;

	bool fStable;

	double fKP, fKD;
};

#endif /* SIMPLECONTROL_H_ */
