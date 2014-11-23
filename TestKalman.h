/*
 * TestKalman.h
 *
 *  Created on: 15 Nov 2014
 *      Author: Nicoas
 */

#ifndef TESTKALMAN_H_
#define TESTKALMAN_H_

class TestKalman {
public:
	TestKalman();
	virtual ~TestKalman();

	void test1();
	void test2();
	float getNormal(float mu, float sigma);
};

#endif /* TESTKALMAN_H_ */
