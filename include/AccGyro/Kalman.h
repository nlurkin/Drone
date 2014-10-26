/*
 * Kalman.h
 *
 *  Created on: 22 Apr 2014
 *      Author: Nicoas
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "Math/VectorFloat.h"
#include "Math/Matrix.h"

template <int I, int O, int C>
class Kalman {
public:
	Kalman(){

	};
	virtual ~Kalman(){

	};

	void setMatF(Matrix<O,O> F){
		this->F = F;
	};
	void setMatH(Matrix<I,O> H){
		this->H = H;
	};
	void setMatB(Matrix<O,C> B){
		this->B = B;
	};
	void setMatQ(Matrix<O,O> Q){
		this->Q = Q;
	};
	void setMatR(Matrix<I,I> R){
		this->R = R;
	};
	void setX0(Matrix<O,1> x){
		this->x_k = x;
	}
	void setP0(Matrix<O,O> p){
		this->P_k = p;
	}

	Matrix<O,1> newMeasure(Matrix<I,1> measure, Matrix<C,1> control){
		predict(control);
		update(measure);
		return this->x_k;
	};

	void predict(Matrix<C,1> u_k = Matrix<C,1>()){
		//float temp[O][O];
		Matrix<O,1> controlPart;
		Matrix<O,1> w_k;
		//x_{k|k-1} = F*x_{k-1|k-1} + w_k
		//get the multivariate normal here
		//w_k =

		if(C>0){
			controlPart = B*u_k;
		}
		else{
			controlPart.zeros();
		}
		x_hat_k = F*x_k + controlPart + w_k;

		//P_{k|k-1} = F*P_{k-1|k-1}*F^T + Q_k
		P_hat_k = F*P_k*F.getTranspose() + Q;

		//x_{k|k-1} = F*x_{k-1|k-1} + w_k
		//temp =
		/*for(int i=0; i<O; i++){
			x_est[i] = w_k[i];
			for(int j=0; j<O; j++){
				x_est[i] += x_upd[j]*F[i][j];
				temp[i][j] = 0;
				for(int k=0; k<O; k++){
					temp[i][j] += F[i][k]*P_upd[k][j];
				}
			}
		}*/

		//P_{k|k-1} = temp*F^T + Q_k
		/*for(int i=0;i<O;i++){
			for(int j=0; j<O; j++){
				P_est[i][j] = Q[i][j];
				for(int k=0;k<O; k++){
					P_est[i][j] += temp[i][k]*FT[k][j];
				}
			}
		}*/
	};
	void update(Matrix<I,1> z_k){
		Matrix<I,1> y;
		//float temp[I][O];
		Matrix<I,I> S;
		Matrix<I,I> SI;
		Matrix<O,I> K;

		//y_k = z_k - H*x_{k|k-1}
		y = z_k - H*x_hat_k;

		//S_k = H*P_{k|k-1}*H^T + R_k
		S = H*P_hat_k*H.getTranspose() + R;

		//K_k = P_{k|k-1}*H^T*S^{-1}
		K = P_hat_k*H.getTranspose()*S.getInverse();

		//x_{k|k} = x_{k|k-1} + K_k*y_k
		x_k = x_hat_k + K*y;

		//P_{k|k} = (I-K_k*H_k)*P_{k|k-1}
		P_k = (Matrix<O,O>::identity() - K*H)*P_hat_k;

		//y_k = z_k - H*x_{k|k-1}
		//temp = H*P_{k|k-1}
		/*for(int i=0; i<I; i++){
			y[i] = z_k[i];
			for(int j=0; j<O; j++){
				y[i] += H[i][j]*x_est[j];
				temp[i][j] = 0;
				for(int k=0; k<O; k++){
					temp[i][j] += H[i][k]*P_est[k][j];
				}
			}
		}*/

		//S_k = temp*H^T + R_k
		/*for(int i=0; i<I; i++){
			for(int j=0; j<I; j++){
				S[i][j] = R[i][j];
				for(int k=0; k<O; k++){
					S[i][j] += temp[i][k]*HT[k][j];
				}
			}
		}*/

		//temp2 = P_{k|k-1}*H^T
		/*float SI[I][I];
		float K[O][I];
		float temp2[O][I];
		for(int i=0; i<O; i++){
			for(int j=0; j<I; j++){
				temp2[i][j] = 0;
				for(int k=0; k<O; k++){
					temp2[i][j] += P_est[i][k]*HT[k][j];
				}
			}
		}*/

		//K_k = temp2*S^{-1}
		//TODO calculer SI
		/*for(int i=0; i<O; i++){
			for(int j=0; j<I; j++){
				K[i][j] = 0;
				for(int k=0; k<I; k++){
					K[i][j] += temp2[i][k]*SI[k][j];
				}
			}
		}*/

		//x_{k|k} = x_{k|k-1} + K_k*y_k
		/*for(int i=0; i<O; i++){
			x_upd[i] = x_est[i];
			for(int j=0; j<I; j++){
				x_upd[i] += K[i][j]*y[j];
			}
		}*/

		//float temp3[O][O];
		//temp3 = I-K_k*H_k
		/*for(int i=0; i<O; i++){
			for(int j=0; j<O; j++){
				temp3[i][j] = 1;
				for(int k=0; k<I; k++){
					temp3[i][j] -= K[i][k]*H[k][j];
				}
			}
		}*/

		//P_{k|k} = temp3*P_{k|k-1}
		/*for(int i=0; i<O; i++){
			for(int j=0; j<O; j++){
				P_upd[i][j] = 0;
				for(int k=0; k<O; k++){
					P_upd[i][j] += temp3[i][k]*P_est[k][j];
				}
			}
		}*/
	};
private:
	//Current state
	Matrix<O,1> x_k;	//Value
	Matrix<O,O> P_k;	//Error

	//Estimated state
	Matrix<O,1> x_hat_k;	//Value after prediction
	Matrix<O,O> P_hat_k;	//Error after prediction

	//Matrices (constants)
	Matrix<O,O> F;
	Matrix<O,C> B;
	Matrix<I,O> H;
	Matrix<O,O> Q;
	Matrix<I,I> R;
};

#endif /* KALMAN_H_ */
