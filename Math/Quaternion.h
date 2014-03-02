/*
 * Quaternion.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

class VectorFloat;

class Quaternion {
public:
	float w;
	float x;
	float y;
	float z;

	Quaternion();
	Quaternion(float nw, float nx, float ny, float nz);
	Quaternion(const Quaternion &q);
	Quaternion(VectorFloat v);

	void set(float nw, float nx, float ny, float nz);

	void print();

	Quaternion operator+(Quaternion q);
	Quaternion operator-(Quaternion q);
	Quaternion operator*(Quaternion q);
	Quaternion operator*(float v);
	Quaternion operator-();
	float &operator[](int i);
	bool operator==(Quaternion q);

	Quaternion conjugate();
	float mag();
	void normalize();

	Quaternion getProduct(Quaternion q);
	Quaternion getNormalized();
	VectorFloat getAngles();
	VectorFloat getVector();
};



#endif /* QUATERNION_H_ */
