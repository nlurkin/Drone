// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
 */

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_
#include <math.h>

class Quaternion {
public:
	float w;
	float x;
	float y;
	float z;

	Quaternion() {
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Quaternion(float nw, float nx, float ny, float nz) {
		w = nw;
		x = nx;
		y = ny;
		z = nz;
	}

	Quaternion(Quaternion q){
		w = q.w;
		x = q.x;
		y = q.y;
		z = q.z;
	}

	Quaternion(VectorFloat v){
		w = 0.0f;
		x = v.x;
		y = v.y;
		z = v.z;
	}

	void set(float nw, float nx, float ny, float nz) {
		w = nw;
		x = nx;
		y = ny;
		z = nz;
	}

	void print(){
		Serial.print("(");
		Serial.print(w);
		Serial.print(",");
		Serial.print(x);
		Serial.print(",");
		Serial.print(y);
		Serial.print(",");
		Serial.print(z);
		Serial.println(")");
	}

	Quaternion operator+(Quaternion q){
		Quaternion r;
		r.w = w + q.w;
		r.x = x + q.x;
		r.y = y + q.y;
		r.z = z + q.z;
		r.normalize();
		return r;
	}

	Quaternion operator-(Quaternion q){
		Quaternion r;
		r.w = w - q.w;
		r.x = x - q.x;
		r.y = y - q.y;
		r.z = z - q.z;
		r.normalize();
		return r;
	}

	Quaternion operator*(Quaternion q){
		return getProduct(q);
	}
	Quaternion getProduct(Quaternion q) {
		Quaternion r;
		r.w = w*q.w - x*q.x - y*q.y - z*q.z;
		r.x = w*q.x + x*q.w - y*q.z + z*q.y;
		r.y = w*q.y + x*q.z + y*q.w - z*q.x;
		r.z = w*q.z - x*q.y + y*q.x + z*q.w;

		return r;
	}

	Quaternion operator*(float v){
		return Quaternion(v*w, v*x, v*y, v*z);
	}

	Quaternion operator-(){
		return Quaternion(-w,-x,-y,-z);
	}

	Quaternion conjugate() {
		return Quaternion(w, -x, -y, -z);
	}

	float mag() {
		return sqrt(w*w + x*x + y*y + z*z);
	}

	void normalize() {
		float m = sqrt(mag());
		w /= m;
		x /= m;
		y /= m;
		z /= m;
	}

	Quaternion getNormalized() {
		Quaternion r(w, x, y, z);
		r.normalize();
		return r;
	}

	VectorFloat getAngles(){
		VectorFloat a;
		a[0] = atan(2*(w*x + y*z)/(1-2*(x*x + y*y)));
		a[1] = asin(2*(w*y - z*x));
		a[2] = atan(2*(w*z + x*y)/(1-2*(y*y + z*z)));
		return a;
	}

	VectorFloat getVector(){
		return VectorFloat(x, y, z);
	}

	float &operator[](int i){
		switch(i){
		case 0:
			return w;
			break;
		case 0:
			return x;
			break;
		case 0:
			return y;
			break;
		case 0:
			return z;
			break;
		}
		return 0;
	}

	bool operator==(Quaternion q){
		return ((w==q.w) && (x==q.x) && (y==q.y) && (z==q.z));
	}

};

class VectorInt16 {
public:
	int16_t x;
	int16_t y;
	int16_t z;

	VectorInt16() {
		x = 0;
		y = 0;
		z = 0;
	}

	VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
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

	void rotate(Quaternion q) {
		VectorFloat r;
		Quaternion p(0, x, y, z);

		p = q*p;
		p = p*q.conjugate();

		x = p.x;
		y = p.y;
		z = p.z;
	}

	VectorInt16 getRotated(Quaternion q) {
		VectorInt16 r(x, y, z);
		r.rotate(q);
		return r;
	}

	void print(){
		Serial.print(x);
		Serial.print(" ");
		Serial.print(y);
		Serial.print(" ");
		Serial.print(z);
		Serial.println("");
	}

};

class VectorFloat {
public:
	float x;
	float y;
	float z;

	VectorFloat() {
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	VectorFloat(float nx, float ny, float nz) {
		x = nx;
		y = ny;
		z = nz;
	}

	VectorFloat(VectorFloat v){
		x = v.x;
		y = v.y;
		z = v.z;
	}

	VectorFloat(Quaternion q) {
		x = q.x;
		y = q.y;
		z = q.z;
	}

	void set(float nx, float ny, float nz){
		x = nx;
		y = ny;
		z = nz;
	}

	float &operator[](int i){
		if(i==0) return x;
		else if(i==1) return y;
		else return z;
	}

	void print(){
		Serial.print("(");
		Serial.print(x);
		Serial.print(",");
		Serial.print(y);
		Serial.print(",");
		Serial.print(z);
		Serial.println(")");
	}

	float mag() {
		return sqrt(x*x + y*y + z*z);
	}

	void normalize() {
		float m = mag();
		x /= m;
		y /= m;
		z /= m;
	}

	VectorFloat getNormalized() {
		VectorFloat r(x, y, z);
		r.normalize();
		return r;
	}

	void rotate(Quaternion q) {
		VectorFloat r;
		Quaternion p(0, x, y, z);

		p = q*p;
		p = p*q.conjugate();

		x = p.x;
		y = p.y;
		z = p.z;
	}

	VectorFloat getRotated(Quaternion q) {
		VectorFloat r(x, y, z);
		r.rotate(q);
		return r;
	}

	VectorFloat operator-(VectorFloat s){
		return VectorFloat(x-s.x, y-s.y, z-s.z);
	}
	VectorFloat operator+(VectorFloat s){
		return VectorFloat(x+s.x, y+s.y, z+s.z);
	}
	VectorFloat operator-(float s){
		return VectorFloat(x-s, y-s, z-s);
	}
	VectorFloat operator+(float s){
		return VectorFloat(x+s, y+s, z+s);
	}
	VectorFloat operator-(){
		return VectorFloat(-x,-y,-z);
	}

	VectorFloat operator*(float s){
		return VectorFloat(x*s, y*s, z*s);
	}
	float operator*(VectorFloat s){
		return (x*s.x + y*s.y + z*s.z);
	}

	VectorFloat operator/(float s){
		return VectorFloat(x/s, y/s, z/s);
	}

	bool operator==(VectorFloat v){
		return ((x==v.x) && (y==v.y) && (z==v.z));
	}

	VectorFloat cross(VectorFloat v){
		v = VectorFloat();
		v[0] = y*v.z-z*v.y;
		v[1] = z*v.x-x*v.z;
		v[2] = x*v.y-y*v.x;
		return v;
	}
};

template<class T, int I, int J>
class MatrixNic{
public:
	MatrixNic(){
		for(int i=0; i<I; i++){
			for(int j=0; j<J; j++){
				data[i][j] = 0.0f;
			}
		}
	};

	T &operator()(int i, int j) {return data[i][j];};
	void operator()(int i, int j, T v) {data[i][j] = v;};

	VectorFloat operator*(VectorFloat in){
		VectorFloat out;
		for(int i=0; i<I; i++){
			for(int j=0; j<J; j++){
				out[i] += data[i][j]*in[j];
			}
		}

		return out;
	};

	void print(){
		for(int i=0; i<I; i++){
			for(int j=0; j<J; j++){
				Serial.print(data[i][j]);
				Serial.print(" ");
			}
			Serial.println("");
		}
	};
private:
	T data[I][J];
};

// Minimal class to replace std::vector
template<typename Data>
class vector {
	size_t d_size; // Stores no. of actually stored objects
	size_t d_capacity; // Stores allocated capacity
	Data *d_data; // Stores data
public:
	vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor
	vector(vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0) {
		d_data = (Data *)malloc(d_capacity*sizeof(Data));
		memcpy(d_data, other.d_data, d_size*sizeof(Data));
	}; // Copy constuctor

	~vector() { free(d_data); }; // Destructor

	void clear(){
		free(d_data);
		d_size = 0;
		d_capacity = 0;
		d_data = 0;
	}
	vector &operator=(vector const &other) {
		free(d_data); d_size = other.d_size;
		d_capacity = other.d_capacity;
		d_data = (Data *)malloc(d_capacity*sizeof(Data));
		memcpy(d_data, other.d_data, d_size*sizeof(Data));
		return *this;
	}; // Needed for memory management

	void push_back(Data const &x) {
		if (d_capacity == d_size) resize();
		d_data[d_size++] = x;
	}; // Adds new value. If needed, allocates more space

	size_t size() const { return d_size; }; // Size getter
	Data const &operator[](size_t idx) const { return d_data[idx]; }; // Const getter
	Data &operator[](size_t idx) { return d_data[idx]; }; // Changeable getter
private:
	void resize() {
		d_capacity = d_capacity ? d_capacity*2 : 1;
		Data *newdata = (Data *)malloc(d_capacity*sizeof(Data));
		memcpy(newdata, d_data, d_size * sizeof(Data));
		free(d_data);
		d_data = newdata;
	};// Allocates double the old space
};


class MotorResponse{
	float Rx, Ry, Rz, Rt;

	float &operator[](int i){
		switch(i){
		case 0:
			return Rx;
			break;
		case 0:
			return Ry;
			break;
		case 0:
			return Rz;
			break;
		case 0:
			return Rt;
			break;
		}
		return 0;
	}

	void set(float x, float y, float z, float t){
		Rx = x;
		Ry = y;
		Rz = z;
		Rt = t;
	}
};


#endif /* _HELPER_3DMATH_H_ */
