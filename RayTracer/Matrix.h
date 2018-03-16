//
// Simple, custom matrix library
//

#include "Util.h"
#pragma once
//Only handles square matrices up to 4x4

#define MAX 16

class Matrix
{
public:
	double vals[MAX] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	int size;
	Matrix(int size) : size(size) {}
	Matrix(int size, double values[]);
	void setValue(int i, int j, double v);
	void setValues(double values[]);
	double getValue(int i, int j);
	Matrix operator/ (double d);
	Matrix getTrans();
	Matrix getMinor(int i, int j);
	double getDet();
	Matrix getCofactor();
	Matrix getInverse();
	//Assumes 4x4 with 3d point
	Point operator*(Point v);
	//Assumes 4x4 with 3d vector
	Vector operator*(Vector v);
};

std::ostream &operator<<(std::ostream &os, Matrix const &m);
