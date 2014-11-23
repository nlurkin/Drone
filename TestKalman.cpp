/*
 * TestKalman.cpp
 *
 *  Created on: 15 Nov 2014
 *      Author: Nicoas
 */

#include "TestKalman.h"
#include "AccGyro/Kalman.h"

TestKalman::TestKalman() {
	// TODO Auto-generated constructor stub

}

TestKalman::~TestKalman() {
	// TODO Auto-generated destructor stub
}

float TestKalman::getNormal(float mu, float sigma){
	static double n2 = 0.0;
	static int n2_cached = 0;
	if (!n2_cached) {
		//choose a point x,y in the unit circle uniformly at random
		double x, y, r;
		do {
		    //scale two random integers to doubles between -1 and 1
			x = 2.0*rand()/RAND_MAX - 1;
			y = 2.0*rand()/RAND_MAX - 1;
		    r = x*x + y*y;
		} while (r == 0.0 || r > 1.0);
		{
		//Apply Box-Muller transform on x, y
		double d = sqrt(-2.0*log(r)/r);
		double n1 = x*d;
		n2 = y*d;
		double result = n1*sigma + mu;
		//scale and translate to get desired mean and standard deviation
		n2_cached = 1;
		return result;
		}
	} else {
		n2_cached = 0;
		return n2*sigma + mu;
	}
}

void TestKalman::test1(){
    Kalman<1, 2, 1> kal;

    float dt = 0.1;
    float sigma_theta = 0.01;

    float hArray[1][2] = {{1., 0}};
    float fArray[2][2] = {{1, dt},{0, 1}};
    float qArray[2][2] = {{dt*dt*dt*dt/4., dt*dt*dt/2.}, {dt*dt*dt/2., dt*dt}};
    float rArray[1][1] = {{sigma_theta*sigma_theta}};
    float bArray[2][1] = {{dt*dt/2.},{dt}};
    kal.setMatH(Matrix<1,2>(hArray));
    kal.setMatF(Matrix<2,2>(fArray));
    kal.setMatQ(Matrix<2,2>(qArray));
    kal.setMatR(Matrix<1,1>(rArray));
    kal.setMatB(Matrix<2,1>(bArray));

    float theta = 0;
    float omega = 0;
    //alpha = 0.2

    float x0Aray[2][1] = {{0},{0}};
    kal.setX0(Matrix<2,1>(x0Aray));
    float p0Aray[2][2] = {{1000, 0},{0, 1000}};
    kal.setP0(Matrix<2,2>(p0Aray));

    /*av = []
    thv = []
    ov = []
    tkv = []
    okv = []
    tv = []*/

    float alpha;
    float newMeasure[1][1];
    float newControl[1][1] = {{10}};
    Matrix<2,1> output;
    for(int t=0; t<=500; t++){
        alpha = getNormal(0,1) + 10;
        omega = omega + alpha*dt;
        theta = theta + omega*dt;

        newMeasure[0][0] = getNormal(theta, sigma_theta);
        output = kal.newMeasure(Matrix<1,1>(newMeasure), Matrix<1,1>(newControl));
        //[[t_k], [v_k]] = kal.newMeasure([[]],[[10]])

        cout << t*dt << " " << alpha << " " << omega << " " << theta << " ";
        cout << output.at(0,0) << " " << output.at(1,0) << " " << output.at(0,0)-alpha << endl;
    }
}

void TestKalman::test2(){
    Kalman<1,2,0> kal;

    float dt = 0.1;
    float sigma_omega = 0.01;

    float hArray[1][2] = {{1., 0}};
    float fArray[2][2] = {{1, dt},{0, 1}};
    float qArray[2][2] = {{dt*dt, 0}, {0, 1}};
    float rArray[1][1] = {{sigma_omega*sigma_omega}};
    //float bArray[2][1] = {{0},{0}};
    kal.setMatH(Matrix<1,2>(hArray));
    kal.setMatF(Matrix<2,2>(fArray));
    kal.setMatQ(Matrix<2,2>(qArray));
    kal.setMatR(Matrix<1,1>(rArray));
    //kal.setMatB(Matrix<2,1>(bArray));

    float omega = 1;
    float alpha = 0.5;

    float x0Aray[2][1] = {{1},{0}};
    kal.setX0(Matrix<2,1>(x0Aray));
    float p0Aray[2][2] = {{1000, 0},{0, 1}};
    kal.setP0(Matrix<2,2>(p0Aray));

    float newMeasure[1][1];
    Matrix<2,1> output;
    for(int t=0; t<=500; t++){
        alpha = alpha + 0.1;
        omega = omega + alpha*dt;
        newMeasure[0][0] = getNormal(omega, sigma_omega);
        output = kal.newMeasure(Matrix<1,1>(newMeasure), Matrix<0,1>());
        cout << t*dt << " " << alpha << " " << omega << " " << 0 << " ";
        cout << output.at(0,0) << " " << output.at(1,0) << " " << output.at(0,0)-alpha << endl;
    }
}
