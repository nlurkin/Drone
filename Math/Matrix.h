/*
 * Matrix.h
 *
 *  Created on: 23 Apr 2014
 *      Author: Nicoas
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "Constants.h"

template <int R, int C>
class Matrix {
public:
	class MRow{
	public:
		MRow(int r, Matrix<R,C>* parent){
			row = r;
			ref = parent;
		};
		float& operator[](int col){
			if(col>=C){
				cout << "Column out of bound" << endl;
			}
			return ref->at(row,col);
		};
	private:
		Matrix<R,C>* ref;
		int row;
	};

	Matrix(){
		invalid = 0.0;
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				data[i][j] = 0.0;
			}
		}
	};
	Matrix(float arr[R][C]){
		invalid = 0.0;
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				data[i][j] = arr[i][j];
			}
		}
	};
	virtual ~Matrix(){};

	MRow operator[](int i){
		return MRow(i, this);
	};
	Matrix<1,C> getRow(int i){
		if(i<R){
			Matrix<1,C> r;
			for(int j=0; j<C; ++j){
				r[0][j] = data[i][j];
			}
			return r;
		}
		else {
			cout << "Row out of bound" << endl;
			return Matrix<1,C>();
		}
	};
	Matrix<R,1> getCol(int i){
		if(i<C){
			Matrix<R,1> r;
			for(int j=0; j<R; ++j){
				r[j][0] = data[j][i];
			}
			return r;
		}
		else {
			cout << "Column out of bound" << endl;
			return Matrix<R,1>();
		}
	};

	Matrix<R,C> operator+(Matrix<R,C> m){
		Matrix<R,C> r;
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				r[i][j] = data[i][j] + m[i][j];
			}
		}

		return r;
	};
	Matrix<R,C> operator-(Matrix<R,C> m){
		Matrix<R,C> r;
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				r[i][j] = data[i][j] - m[i][j];
			}
		}

		return r;
	};

	Matrix<R,C>& operator+=(Matrix<R,C> m){
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				data[i][j] += m[i][j];
			}
		}
		return *this;
	};
	Matrix<R,C>& operator-=(Matrix<R,C> m){
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				data[i][j] -= m[i][j];
			}
		}
		return *this;
	};

	float& at(int i, int j){
		if(i>=R){
			cout << "Row out of bound" << endl;
			return invalid;
		}
		if(j>=C){
			cout << "Column out of bound" << endl;
			return invalid;
		}
		return data[i][j];
	};
	template <int O>
	Matrix<R,O> operator*(Matrix<C,O> m){
		Matrix<R,O> r;

		for(int i=0; i<R; ++i){
			for(int j=0; j<O; ++j){
				for(int k=0; k<C; ++k)
					r[i][j] += data[i][k]*m[k][j];
			}
		}

		return r;
	};

	Matrix<C,R> getTranspose(){
		Matrix<C,R> r;

		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				r[j][i] = data[i][j];
			}
		}
		return r;
	};

	static Matrix<R,C> identity(){
		if(R!=C){
			cout << "Matrix::identity only valid for square matrices" << endl;
			return Matrix<R,C>();
		}
		Matrix<R,C> r;
		for(int i=0; i<R; ++i){
			r[i][i] = 1.;
		}

		return r;
	};
	Matrix<R,C> getInverse(){
		if(R!=C){
			cout << "Matrix::getInverse only valid for square matrices" << endl;
			return Matrix<R,C>();
		}
		int pivrow = 0;			//keeps track of current pivot row
		vector<int> pivrows;	//keeps track of rows swaps to undo at end ??? ou 1,N
		int m;

		Matrix<R,R> inv(data);

		for(int k=0; k<R; ++k){
			//find pivot row, the row with biggest entry in current column
			 m = inv.maxCol(k, pivrow);

			//check for singular matrix
			if(m == 0.0){
				cout << "Inversion failed due to singular matrix" << endl;
				return 0;
			}
			if(pivrow != k){
				inv.swap(k, pivrow);
			}

			pivrows.push_back(pivrow);	//record row swap (even if no swap happened)
			inv.divideRow(k, m);

			//Now eliminate all other entries in this column
			for(int i=0; i<R; ++i){
				if(i != k){
					inv.substractRow(i, inv[i][k], k);
				}
			}


			//Done, now need to undo pivot row swaps by doing column swaps in reverse order
			for(int k=R-1; k>=0; --k){
				if(pivrows[k] != k){
					inv.swapCol(k, pivrows[k]);
				}
			}
		}
		return inv;
	};

	void swap(int i, int j){
		for(int k=0; k<C; ++k){
			swap(data[i][k], data[j][k]);
		}
	};
	void swapCol(int i, int j){
		for(int k=0; k<R; ++k){
			swap(data[k][i], data[k][j]);
		}
	};
	void divideRow(int i, float d){
		for(int k=0; k<C; ++k){
			data[i][k] /= d;
		}
	};
	void substractRow(int i, float m, int j){
		//L1 = L1 - m*L2
		for(int k=0; k<C; ++k){
			data[i][k] = data[i][k] - m*data[j][k];
		}
	};
	float maxCol(int i, int& index){
		float max = data[0][i];
		index = 0;
		for(int k=1; k<R; ++k){
			if(max < data[k][i]){
				max = data[k][i];
				index = k;
			}
		}
		return max;
	};

	void print(){
		for(int i=0; i<R; ++i){
			for(int j=0; j<C; ++j){
				cout << data[i][j] << "\t";
			}
			cout << endl;
		}
	};

private:
	float data[R][C];
	float invalid;
};

#endif /* MATRIX_H_ */
