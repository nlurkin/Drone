/*
 * Kalman.cpp
 *
 *  Created on: 22 Apr 2014
 *      Author: Nicoas
 */

#include "Kalman.h"

template <int O, int I>
Kalman<O,I>::Kalman() {
	// TODO Auto-generated constructor stub

}

template <int O, int I>
Kalman<O,I>::~Kalman() {
	// TODO Auto-generated destructor stub
}

template <int O, int I>
void Kalman<O,I>::setup(float dt){
/*	F[0][0] = 1;
	F[0][1] = dt;
	F[0][2] = dt*dt/2.;
	F[1][0] = 0;
	F[1][1] = 1;
	F[1][2] = dt;
	F[2][0] = 0;
	F[3][1] = 0;
	F[3][2] = 1;

	FT[0][0] = 1;
	FT[0][1] = 0;
	FT[0][2] = 0;
	FT[1][0] = dt;
	FT[1][1] = 1;
	FT[1][2] = 0;
	FT[2][0] = dt*dt/2.;
	FT[3][1] = dt;
	FT[3][2] = 1;

	H[0][0] = 1;
	H[0][1] = 0;
	H[0][2] = 0;
	H[1][0] = 0;
	H[1][1] = 1;
	H[1][2] = 0;
	H[2][0] = 0;
	H[3][1] = 0;
	H[3][2] = 1;*/
}

template <int O, int I>
void Kalman<O,I>::predict(){

	//float temp[O][O];

	//x_{k|k-1} = F*x_{k-1|k-1} + w_k
	x_est = F*x_upd + w_k;

	//P_{k|k-1} = F*P_{k-1|k-1}*F^T + Q_k
	P_est = F*P_upd*F.getTranspose() + Q;

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
}

template <int O, int I>
void Kalman<O,I>::update(){
	float y[I];
	//float temp[I][O];
	float S[I][I];
	float SI[I][I];
	float K[O][I];

	//y_k = z_k - H*x_{k|k-1}
	y = z_k - H*x_est;

	//S_k = H*P_{k|k-1}*H^T + R_k
	S = H*P_est*H.getTranspose() + R;

	//K_k = P_{k|k-1}*H^T*S^{-1}
	K = P_est*H.getTranspose()*S.getInverse();

	//x_{k|k} = x_{k|k-1} + K_k*y_k
	x_upd = x_est + K*y;

	//P_{k|k} = (I-K_k*H_k)*P_{k|k-1}
	P_upd = (Matrix<O,O>::identity() - K*H)*P_est;

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
}
