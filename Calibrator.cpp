/*
 * Calibrator.cpp
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#include "Calibrator.h"
#include "MatrixMath.h"

Calibrator::Calibrator() {
	dataPoints = 0;
	mass = 1;
}

Calibrator::~Calibrator() {
	// TODO Auto-generated destructor stub
}

void Calibrator::newPoint(int motor, float p, VectorFloat omega, VectorFloat alpha, VectorFloat acceleration, Quaternion q) {
	fP[motor].push_back(p);
	fOmega[motor].push_back(omega);
	fAlpha[motor].push_back(alpha);
	fA[motor].push_back(acceleration);
	fQ[motor].push_back(q);
}

bool Calibrator::calibrateI(int motor){
	if(fP[motor].size()<2) return false;
	VectorFloat I;

	//This works if the time difference between both measurement is big enough and time step small enough and avoid angles close to k*pi
	I[0] = (fAlpha[motor][1][0]-fAlpha[motor][0][0])/(fOmega[motor][1][1]*fOmega[motor][1][2]-fOmega[motor][0][1]*fOmega[motor][0][2]);
	I[1] = (fAlpha[motor][1][1]-fAlpha[motor][0][1])/(fOmega[motor][1][0]*fOmega[motor][1][2]-fOmega[motor][0][0]*fOmega[motor][0][2]);
	I[2] = (fAlpha[motor][1][2]-fAlpha[motor][0][2])/(fOmega[motor][1][0]*fOmega[motor][1][1]-fOmega[motor][0][0]*fOmega[motor][0][1]);
	//print "Estimated I1 " + str(I1)
	//print "Estimated I2 " + str(I2)
	//print "Estimated I3 " + str(I3)
	if(fI.mag() == 0) fI = I;
	else fI = fI+I;

	dataPoints += 1;

	return true;
}

bool Calibrator::calibrate() {

	return true;
}

bool Calibrator::calibrateR(int motor) {
	VectorFloat I = getAveragedI();
	VectorFloat a1, a2, g1, g2, g;
	VectorFloat Rt;

	//This works if I is well known and simulation step is small enough so that the difference between oldomega and omega is small
	fR[motor][0] = ((fAlpha[motor][0][0] - fOmega[motor][0][1]*fOmega[motor][0][2]*I[0])/fP[motor][0])*fIAxis[0];
	fR[motor][1] = ((fAlpha[motor][0][1] - fOmega[motor][0][0]*fOmega[motor][0][2]*I[1])/fP[motor][0])*fIAxis[1];
	fR[motor][2] = ((fAlpha[motor][0][2] - fOmega[motor][0][0]*fOmega[motor][0][1]*I[2])/fP[motor][0])*fIAxis[2];
	//print "Estimated Rx " + str(Rx)
	//print "Estimated Ry " + str(Ry)
	//print "Estimated Ry " + str(Rz)

	a1 = fA[motor][0].rotate(fQ[motor][0].conjugate());
	a2 = fA[motor][1].rotate(fQ[motor][1].conjugate());

	g[2] = 9.81;
	g1 = g.rotate(fQ[motor][0].conjugate());
	g2 = g.rotate(fQ[motor][1].conjugate());

	//Ok but needs to be with very small velocity to neglect friction
	Rt = (mass*(a1- a2) + mass*(g1-g2))/(fP[motor][0]-fP[motor][1]);

	fR[motor][3] = Rt[2];

	return true;
}

VectorFloat Calibrator::getAveragedI() {
	return fI/float(dataPoints);
}

void Calibrator::finalizeI() {
	VectorFloat I = getAveragedI();
	fIAxis[0] = 0.177;
	fIAxis[1] = fIAxis[0]*(I[0]*I[1]*I[2] + I[1]+1-I[2])/(I[1]+1);
	fIAxis[2] = fIAxis[0]*(1+I[0]*I[1])/(I[1]+1);
}

VectorFloat Calibrator::getIAxis(){
	return fIAxis;
}

MotorResponse Calibrator::getR(int i) {
	return fR[i];
}

void Calibrator::clearPoints(){
	fP[0].clear();
	fP[1].clear();
	fP[2].clear();
	fP[3].clear();
	fOmega[0].clear();
	fOmega[1].clear();
	fOmega[2].clear();
	fOmega[3].clear();
	fAlpha[0].clear();
	fAlpha[1].clear();
	fAlpha[2].clear();
	fAlpha[3].clear();
	fA[0].clear();
	fA[1].clear();
	fA[2].clear();
	fA[3].clear();
	fQ[0].clear();
	fQ[1].clear();
	fQ[2].clear();
	fQ[3].clear();
}
