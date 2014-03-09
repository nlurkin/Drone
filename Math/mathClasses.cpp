#include "mathClasses.h"

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

template<class T>
void VectorInt<T>::rotate(Quaternion q) {
	VectorFloat r;
	Quaternion p(0, x, y, z);

	p = q*p;
	p = p*q.conjugate();

	x = p.x;
	y = p.y;
	z = p.z;
}

template<class T>
VectorInt<T> VectorInt<T>::getRotated(Quaternion q) {
	VectorInt<T> r(x, y, z);
	r.rotate(q);
	return r;
}

template<class T>
void VectorInt<T>::print(){
	Serial.print(x);
	Serial.print(" ");
	Serial.print(y);
	Serial.print(" ");
	Serial.print(z);
	Serial.println("");
}


