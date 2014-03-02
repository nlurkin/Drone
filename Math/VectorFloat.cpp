/*
 * VectorFloat.cpp
 *
 *  Created on: 2 Mar 2014
 *      Author: Nicoas
 */

#include "VectorFloat.h"
#include "Quaternion.h"
#include <Arduino.h>
#include <math.h>

VectorFloat::VectorFloat() {
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

VectorFloat::VectorFloat(float nx, float ny, float nz) {
	x = nx;
	y = ny;
	z = nz;
}

VectorFloat::VectorFloat(const VectorFloat &v){
	x = v.x;
	y = v.y;
	z = v.z;
}

VectorFloat::VectorFloat(Quaternion q) {
	x = q.x;
	y = q.y;
	z = q.z;
}

void VectorFloat::set(float nx, float ny, float nz){
	x = nx;
	y = ny;
	z = nz;
}

float &VectorFloat::operator[](int i){
	if(i==0) return x;
	else if(i==1) return y;
	else return z;
}

void VectorFloat::print(){
	Serial.print("(");
	Serial.print(x);
	Serial.print(",");
	Serial.print(y);
	Serial.print(",");
	Serial.print(z);
	Serial.println(")");
}

float VectorFloat::mag() {
	return sqrt(x*x + y*y + z*z);
}

void VectorFloat::normalize() {
	float m = mag();
	x /= m;
	y /= m;
	z /= m;
}

VectorFloat VectorFloat::getNormalized() {
	VectorFloat r(x, y, z);
	r.normalize();
	return r;
}

VectorFloat &VectorFloat::rotate(Quaternion q) {
	VectorFloat r;
	Quaternion p(0, x, y, z);

	p = q*p;
	p = p*q.conjugate();

	x = p.x;
	y = p.y;
	z = p.z;

	return *this;
}

VectorFloat VectorFloat::getRotated(Quaternion q) {
	VectorFloat r(x, y, z);
	r.rotate(q);
	return r;
}

VectorFloat VectorFloat::operator-(VectorFloat s){
	return VectorFloat(x-s.x, y-s.y, z-s.z);
}
VectorFloat VectorFloat::operator+(VectorFloat s){
	return VectorFloat(x+s.x, y+s.y, z+s.z);
}
VectorFloat VectorFloat::operator-(float s){
	return VectorFloat(x-s, y-s, z-s);
}
VectorFloat VectorFloat::operator+(float s){
	return VectorFloat(x+s, y+s, z+s);
}
VectorFloat VectorFloat::operator-(){
	return VectorFloat(-x,-y,-z);
}

VectorFloat VectorFloat::operator*(float s){
	return VectorFloat(x*s, y*s, z*s);
}
float VectorFloat::operator*(VectorFloat s){
	return (x*s.x + y*s.y + z*s.z);
}

VectorFloat VectorFloat::operator/(float s){
	return VectorFloat(x/s, y/s, z/s);
}

bool VectorFloat::operator==(VectorFloat v){
	return ((x==v.x) && (y==v.y) && (z==v.z));
}

VectorFloat VectorFloat::cross(VectorFloat v){
	v = VectorFloat();
	v[0] = y*v.z-z*v.y;
	v[1] = z*v.x-x*v.z;
	v[2] = x*v.y-y*v.x;
	return v;
}



