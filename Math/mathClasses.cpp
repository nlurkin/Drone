#include "mathClasses.h"


VectorInt16::VectorInt16() {
	x = 0;
	y = 0;
	z = 0;
}

VectorInt16::VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
	x = nx;
	y = ny;
	z = nz;
}

/*float getMagnitude() {
		return sqrt(x*x + y*y + z*z);
	}*/

/*void normalize() {
		float m = getMagnitude();
		x /= m;
		y /= m;
		z /= m;
	}*/

/*VectorInt16 getNormalized() {
		VectorInt16 r(x, y, z);
		r.normalize();
		return r;
	}*/

void VectorInt16::rotate(Quaternion q) {
	VectorFloat r;
	Quaternion p(0, x, y, z);

	p = q*p;
	p = p*q.conjugate();

	x = p.x;
	y = p.y;
	z = p.z;
}

VectorInt16 VectorInt16::getRotated(Quaternion q) {
	VectorInt16 r(x, y, z);
	r.rotate(q);
	return r;
}

void VectorInt16::print(){
	Serial.print(x);
	Serial.print(" ");
	Serial.print(y);
	Serial.print(" ");
	Serial.print(z);
	Serial.println("");
}


