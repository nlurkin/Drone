#include "Math/Quaternion.h"
/*
 * Quaternion.cpp
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#include "Quaternion.h"
#include <math.h>
#include <Arduino.h>
#include "VectorFloat.h"
#include "Constants.h"

Quaternion::Quaternion() {
	w = 1.0f;
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

Quaternion::Quaternion(float nw, float nx, float ny, float nz) {
	w = nw;
	x = nx;
	y = ny;
	z = nz;
}

Quaternion::Quaternion(const Quaternion &q){
	w = q.w;
	x = q.x;
	y = q.y;
	z = q.z;
}

Quaternion::Quaternion(VectorFloat v){
	w = 0.0f;
	x = v.x;
	y = v.y;
	z = v.z;
}

void Quaternion::set(float nw, float nx, float ny, float nz) {
	w = nw;
	x = nx;
	y = ny;
	z = nz;
}

void Quaternion::print(){
	cout << "(" << w << "," << x << "," << y << "," << z << ")" << endl;
}

Quaternion Quaternion::operator+(Quaternion q){
	Quaternion r;
	r.w = w + q.w;
	r.x = x + q.x;
	r.y = y + q.y;
	r.z = z + q.z;
	r.normalize();
	return r;
}

Quaternion Quaternion::operator-(Quaternion q){
	Quaternion r;
	r.w = w - q.w;
	r.x = x - q.x;
	r.y = y - q.y;
	r.z = z - q.z;
	r.normalize();
	return r;
}

Quaternion Quaternion::operator*(Quaternion q){
	return getProduct(q);
}
Quaternion Quaternion::getProduct(Quaternion q) {
	Quaternion r;
	r.w = w*q.w - x*q.x - y*q.y - z*q.z;
	r.x = w*q.x + x*q.w - y*q.z + z*q.y;
	r.y = w*q.y + x*q.z + y*q.w - z*q.x;
	r.z = w*q.z - x*q.y + y*q.x + z*q.w;

	return r;
}

Quaternion Quaternion::operator*(float v){
	return Quaternion(v*w, v*x, v*y, v*z);
}

Quaternion Quaternion::operator-(){
	return Quaternion(-w,-x,-y,-z);
}

Quaternion Quaternion::conjugate() {
	return Quaternion(w, -x, -y, -z);
}

float Quaternion::mag() {
	return sqrt(w*w + x*x + y*y + z*z);
}

void Quaternion::normalize() {
	float m = sqrt(mag());
	w /= m;
	x /= m;
	y /= m;
	z /= m;
}

Quaternion Quaternion::getNormalized() {
	Quaternion r(w, x, y, z);
	r.normalize();
	return r;
}

VectorFloat Quaternion::getAngles(){
	VectorFloat a;
	a[0] = atan(2*(w*x + y*z)/(1-2*(x*x + y*y)));
	a[1] = asin(2*(w*y - z*x));
	a[2] = atan(2*(w*z + x*y)/(1-2*(y*y + z*z)));
	return a;
}

VectorFloat Quaternion::getVector(){
	return VectorFloat(x, y, z);
}

float &Quaternion::operator[](int i){
	switch(i){
	case 0:
		return w;
		break;
	case 1:
		return x;
		break;
	case 2:
		return y;
		break;
	case 3:
		return z;
		break;
	}
	return w;
}

bool Quaternion::operator==(Quaternion q){
	return ((w==q.w) && (x==q.x) && (y==q.y) && (z==q.z));
}
