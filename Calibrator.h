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

	bool newPoint(int motor, float p, VectorFloat omega, VectorFloat alpha);
	bool calibrate();

	void clearPoints();

private:
	bool calibrateIndividual(int motor);

	MatrixNic<int, 3, 3> fI;
	VectorFloat R[4];

	vector<int> fP[4];
	vector<VectorFloat> fOmega[4], fAlpha[4];
};

#endif /* CALIBRATOR_H_ */
