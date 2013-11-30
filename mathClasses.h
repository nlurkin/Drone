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

	Quaternion getProduct(Quaternion q) {
		// Quaternion multiplication is defined by:
		//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
		//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
		//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
		//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
		return Quaternion(
				w*q.w - x*q.x - y*q.y - z*q.z,  // new w
				w*q.x + x*q.w + y*q.z - z*q.y,  // new x
				w*q.y - x*q.z + y*q.w + z*q.x,  // new y
				w*q.z + x*q.y - y*q.x + z*q.w); // new z
	}

	Quaternion getConjugate() {
		return Quaternion(w, -x, -y, -z);
	}

	float getMagnitude() {
		return sqrt(w*w + x*x + y*y + z*z);
	}

	void normalize() {
		float m = getMagnitude();
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

	Quaternion operator*(Quaternion q){
		return getProduct(q);
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

	float getMagnitude() {
		return sqrt(x*x + y*y + z*z);
	}

	void normalize() {
		float m = getMagnitude();
		x /= m;
		y /= m;
		z /= m;
	}

	VectorInt16 getNormalized() {
		VectorInt16 r(x, y, z);
		r.normalize();
		return r;
	}

	void rotate(Quaternion *q) {
		// http://www.cprogramming.com/tutorial/3d/quaternions.html
		// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
		// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
		// ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

		// P_out = q * P_in * conj(q)
		// - P_out is the output vector
		// - q is the orientation quaternion
		// - P_in is the input vector (a*aReal)
		// - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
		Quaternion p(0, x, y, z);

		// quaternion multiplication: q * p, stored back in p
		p = q -> getProduct(p);

		// quaternion multiplication: p * conj(q), stored back in p
		p = p.getProduct(q -> getConjugate());

		// p quaternion is now [0, x', y', z']
		x = p.x;
		y = p.y;
		z = p.z;
	}

	VectorInt16 getRotated(Quaternion *q) {
		VectorInt16 r(x, y, z);
		r.rotate(q);
		return r;
	}
};

class VectorFloat {
public:
	float x;
	float y;
	float z;

	VectorFloat() {
		x = 0;
		y = 0;
		z = 0;
	}

	VectorFloat(float nx, float ny, float nz) {
		x = nx;
		y = ny;
		z = nz;
	}

	VectorFloat(Quaternion &q) {
		x = q.x;
		y = q.y;
		z = q.z;
	}

	float getMagnitude() {
		return sqrt(x*x + y*y + z*z);
	}

	void normalize() {
		float m = getMagnitude();
		x /= m;
		y /= m;
		z /= m;
	}

	VectorFloat getNormalized() {
		VectorFloat r(x, y, z);
		r.normalize();
		return r;
	}

	void rotate(Quaternion *q) {
		Quaternion p(0, x, y, z);

		// quaternion multiplication: q * p, stored back in p
		p = q -> getProduct(p);

		// quaternion multiplication: p * conj(q), stored back in p
		p = p.getProduct(q -> getConjugate());

		// p quaternion is now [0, x', y', z']
		x = p.x;
		y = p.y;
		z = p.z;
	}

	VectorFloat getRotated(Quaternion *q) {
		VectorFloat r(x, y, z);
		r.rotate(q);
		return r;
	}

	VectorFloat operator*(float s){
		VectorFloat r(x,y,z);
		r.x = r.x * s;
		r.y = r.y * s;
		r.z = r.z * s;
		return r;
	}

	VectorFloat operator/(float s){
		VectorFloat r(x,y,z);
		r.x = r.x / s;
		r.y = r.y / s;
		r.z = r.z / s;
		return r;
	}

	VectorFloat operator-(float s){
		VectorFloat r(x,y,z);
		r.x = r.x - s;
		r.y = r.y - s;
		r.z = r.z - s;
		return r;
	}
	VectorFloat operator+(float s){
		VectorFloat r(x,y,z);
		r.x = r.x + s;
		r.y = r.y + s;
		r.z = r.z + s;
		return r;
	}

	VectorFloat operator-(VectorFloat s){
		VectorFloat r(x,y,z);
		r.x = r.x - s.x;
		r.y = r.y - s.y;
		r.z = r.z - s.z;
		return r;
	}
	VectorFloat operator+(VectorFloat s){
		VectorFloat r(x,y,z);
		r.x = r.x + s.z;
		r.y = r.y + s.y;
		r.z = r.z + s.z;
		return r;
	}

	VectorFloat operator-(){
		VectorFloat r(x,y,z);
		r.x = -r.x;
		r.y = -r.y;
		r.z = -r.z;
		return r;
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

	MatrixNic operator()(int i, int j) {return data[i][j];};
	void operator()(int i, int j, T v) {data[i][j] = v;};

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

#endif /* _HELPER_3DMATH_H_ */
