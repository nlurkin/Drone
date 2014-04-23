/*
 * Kalman.h
 *
 *  Created on: 22 Apr 2014
 *      Author: Nicoas
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "VectorFloat.h"
#include "Matrix.h"

template <int O, int I>
class Kalman {
public:
	Kalman();
	virtual ~Kalman();

	void setup(float dt);
	void predict();
	void update();
private:
	//Current state
	Matrix<O,1> x_k;	//Value
	Matrix<O,O> P_k;	//Error
	Matrix<I,1> z_k;	//Measurement
	Matrix<O,1> w_k;	//Noise
	Matrix<I,1> v_k;	//Measurement noise

	//Estimated state
	Matrix<O,1> x_est;	//Value after prediction
	Matrix<O,O> P_est;	//Error after prediction

	//Updated state
	Matrix<O,1> x_upd;
	Matrix<O,O> P_upd;

	//Matrices (constants)
	Matrix<O,O> F;
	Matrix<O,O> Q;
	Matrix<I,O> H;
	Matrix<I,I> R;

	//Transposed matrices
	Matrix<O,O> FT;
	Matrix<O,I> HT;
};

#endif /* KALMAN_H_ */
