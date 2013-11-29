/*
 * Calibrator.cpp
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#include "Calibrator.h"
#include "MatrixMath.h"

Calibrator::Calibrator() {
	// TODO Auto-generated constructor stub

}

Calibrator::~Calibrator() {
	// TODO Auto-generated destructor stub
}

bool Calibrator::newPoint(int motor, float p, VectorFloat omega, VectorFloat alpha) {
	fP[motor].push_back(p);
	fOmega[motor].push_back(omega);
	fAlpha[motor].push_back(alpha);

	return calibrateIndividual();
}

bool Calibrator::calibrate() {

	fP.clear();
}

bool Calibrator::calibrateIndividual() {
	if(fP.size()<12) return false;

	fvec x(12);
	fmat a(12, 12);
	fvec b(12);

	x = solve(a, b);
}
