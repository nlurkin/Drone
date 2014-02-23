/*
 * Calibrator.h
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include <Arduino.h>
#include "mathClasses.h"
#include "MatrixMath.h"

class Calibrator {
public:
	Calibrator();
	virtual ~Calibrator();

	void newPoint(int motor, float p, VectorFloat omega, VectorFloat alpha, VectorFloat acceleration, Quaternion q);
	bool calibrate();
	bool calibrateI();
	bool calibrateR();
	VectorFloat getAveragedI();
	VectorFloat getIAxis();
	void finalizeI();
	MotorResponse getR(int i);

	void clearPoints();

private:
	VectorFloat fI, fIAxis;
	MotorResponse fR[4];

	vector<int> fP[4];
	vector<VectorFloat> fOmega[4], fAlpha[4], fA[4];
	vector<Quaternion> fQ[4];

	int dataPoints;
	float mass;
};

#endif /* CALIBRATOR_H_ */
