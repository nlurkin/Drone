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

	return calibrateIndividual(motor);
}

bool Calibrator::calibrate() {

	fP.clear();

	return true;
}

bool Calibrator::calibrateIndividual(int motor) {
	if(fP.size()<12) return false;

	float a[12][12];
	float b[12];
	float x[12];

	for(int i=0; i<4; i++){
		for(int j=0; j<3; j++){
			b[j+3*i] = 0;
			x[j+3*i] = 0;
		}
		a[0+3*i][0] = fAlpha[motor][i].x;
		a[0+3*i][1] = fAlpha[motor][i].y;
		a[0+3*i][2] = fAlpha[motor][i].z;
		a[0+3*i][3] = fAlpha[motor][i].x*fOmega[motor][i].z;
		a[0+3*i][4] = fAlpha[motor][i].y*fOmega[motor][i].z;
		a[0+3*i][5] = fAlpha[motor][i].z*fOmega[motor][i].z;
		a[0+3*i][6] = fAlpha[motor][i].x*fOmega[motor][i].y;
		a[0+3*i][7] = fAlpha[motor][i].y*fOmega[motor][i].y;
		a[0+3*i][8] = fAlpha[motor][i].z*fOmega[motor][i].y;
		a[0+3*i][9] = fP[motor][i];
		a[0+3*i][10] = 0;
		a[0+3*i][11] = 0;
		a[1+3*i][0] = fAlpha[motor][i].x*fOmega[motor][i].z;
		a[1+3*i][1] = fAlpha[motor][i].y*fOmega[motor][i].z;
		a[1+3*i][2] = fAlpha[motor][i].z*fOmega[motor][i].z;
		a[1+3*i][3] = fAlpha[motor][i].x;
		a[1+3*i][4] = fAlpha[motor][i].y;
		a[1+3*i][5] = fAlpha[motor][i].z;
		a[1+3*i][6] = fAlpha[motor][i].x*fOmega[motor][i].x;
		a[1+3*i][7] = fAlpha[motor][i].y*fOmega[motor][i].x;
		a[1+3*i][8] = fAlpha[motor][i].z*fOmega[motor][i].x;
		a[1+3*i][9] = 0;
		a[1+3*i][10] = fP[motor][i];
		a[1+3*i][11] = 0;
		a[2+3*i][0] = fAlpha[motor][i].x*fOmega[motor][i].y;
		a[2+3*i][1] = fAlpha[motor][i].y*fOmega[motor][i].y;
		a[2+3*i][2] = fAlpha[motor][i].z*fOmega[motor][i].y;
		a[2+3*i][3] = fAlpha[motor][i].x*fOmega[motor][i].x;
		a[2+3*i][4] = fAlpha[motor][i].y*fOmega[motor][i].x;
		a[2+3*i][5] = fAlpha[motor][i].z*fOmega[motor][i].x;
		a[2+3*i][6] = fAlpha[motor][i].x;
		a[2+3*i][7] = fAlpha[motor][i].y;
		a[2+3*i][8] = fAlpha[motor][i].z;
		a[2+3*i][9] = 0;
		a[2+3*i][10] = 0;
		a[2+3*i][11] = fP[motor][i];
	}

	if(Matrix.Invert((float*)a, 12)){
		Matrix.Multiply((float*)a, (float*)b, 12, 12, 12, (float*)x);
		fI(0,0) = x[0];
		fI(0,1) = x[1];
		fI(0,2) = x[2];
		fI(1,0) = x[3];
		fI(1,1) = x[4];
		fI(1,2) = x[5];
		fI(2,0) = x[6];
		fI(2,1) = x[7];
		fI(2,2) = x[8];
		R[motor].x = x[9];
		R[motor].y = x[10];
		R[motor].z = x[11];

		return true;
	}
	return false;
}
