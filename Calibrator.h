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

class Calibrator {
public:
	Calibrator();
	virtual ~Calibrator();

	bool newPoint(int motor, float p, VectorFloat omega, VectorFloat alpha);
	bool calibrate();

private:
	bool calibrateIndividual();

	fmat fI;
	vector<VectorFloat> R;

	vector< vector<int>> fP;
	vector< vector<VectorFloat>> fOmega, fAlpha;
};

#endif /* CALIBRATOR_H_ */
