/*
 * VectorFloat.h
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#ifndef VECTORFLOAT_H_
#define VECTORFLOAT_H_

class Quaternion;

class VectorFloat {
public:
	float x;
	float y;
	float z;

	VectorFloat();
	VectorFloat(float nx, float ny, float nz);
	VectorFloat(const VectorFloat &v);
	VectorFloat(Quaternion q);

	void set(float nx, float ny, float nz);

	void print();

	float mag();
	void normalize();
	VectorFloat &rotate(Quaternion q);
	VectorFloat cross(VectorFloat v);

	VectorFloat getNormalized();
	VectorFloat getRotated(Quaternion q);

	VectorFloat operator-(VectorFloat s);
	VectorFloat operator+(VectorFloat s);
	VectorFloat operator-(float s);
	VectorFloat operator+(float s);
	VectorFloat operator-();
	VectorFloat operator*(float s);
	float operator*(VectorFloat s);
	VectorFloat operator/(float s);
	bool operator==(VectorFloat v);
	float &operator[](int i);
};



#endif /* VECTORFLOAT_H_ */
