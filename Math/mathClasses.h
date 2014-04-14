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
#include "mathClasses.h"
#include <Arduino.h>
#include "Quaternion.h"
#include "VectorFloat.h"

template<class T>
class VectorInt {
public:
	T x;
	T y;
	T z;

	VectorInt(){
		x = 0;
		y = 0;
		z = 0;
	};

	VectorInt(T nx, T ny, T nz):x(nx),y(ny),z(nz) {};

	void rotate(Quaternion q);
	VectorInt<T> getRotated(Quaternion q);

	void print(){
		Serial.print(x);
		Serial.print(" ");
		Serial.print(y);
		Serial.print(" ");
		Serial.print(z);
		Serial.println("");
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
public:
	float &operator[](int i){
		switch(i){
		case 0:
			return Rx;
			break;
		case 1:
			return Ry;
			break;
		case 2:
			return Rz;
			break;
		case 3:
			return Rt;
			break;
		}
		return Rx;
	}

	void set(float x, float y, float z, float t){
		Rx = x;
		Ry = y;
		Rz = z;
		Rt = t;
	}
};


#endif /* _HELPER_3DMATH_H_ */
