/*
 * AttitudeLoop.h
 *
 *  Created on: 29 nov. 2013
 *      Author: Nicolas
 */

#ifndef ATTITUDELOOP_H_
#define ATTITUDELOOP_H_

#include <arduino/Arduino.h>
#include "Math/mathClasses.h"

typedef struct motorFactor{
	int m1;
	int m2;
	int m3;
	int m4;
} motorFactor_t;

typedef struct motorConstants{
	double Rx, Ry, Rz;
	double RT;
} motorConstants_t;

class AttitudeLoop {
public:
	AttitudeLoop();
	virtual ~AttitudeLoop();

	VectorFloat ComputePP(Quaternion qM, VectorFloat omegaM);
	motorFactor_t Compute(Quaternion qM, VectorFloat omegaM);

	const Quaternion& getQRef() const {return fQRef;}
	float getPq() const {return fPQ;}
	float getPOmega() const {return fPOmega;}
	void setQRef(const Quaternion& qRef) {fQRef = qRef;}
	void setP(float pQ, float pOmega) {
		fPQ = pQ;
		fPOmega = pOmega;
	}
	const VectorFloat& getTorque() const {return fTorque;}

	const MatrixNic<float, 3, 3>& getI() const {return fI;}
	void setI(const MatrixNic<float, 3, 3>& i) {fI = i;}
	void setM(int motor, double Rx, double Ry, double Rz, double RT);

	void printI();
private:
	Quaternion fQRef;
	float fPQ, fPOmega;
	VectorFloat fTorque;

	MatrixNic<float, 3, 3> fI;
	motorConstants_t fM1, fM2, fM3, fM4;
};

#endif /* ATTITUDELOOP_H_ */
