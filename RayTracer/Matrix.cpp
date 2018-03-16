#include "stdafx.h"
#include "Matrix.h"

Matrix::Matrix(int size, double values[])
{
	this->size = size;
	for (int i = 0; i < size*size; i++) {
		vals[i] = values[i];
	}
}

void Matrix::setValue(int i, int j, double v)
{
	vals[i*size + j] = v;
}

void Matrix::setValues(double values[])
{
	for (int i = 0; i < size*size; i++) {
		vals[i] = values[i];
	}
}

double Matrix::getValue(int i, int j)
{
	return vals[i*size+j];
}

Matrix Matrix::operator/(double d)
{
	Matrix m = Matrix(size);
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			m.setValue(i, j, getValue(i, j)/d);
		}
	}
	return m;
}

Matrix Matrix::getTrans()
{
	Matrix m = Matrix(size);
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			m.setValue(i, j, getValue(j, i));
		}
	}
	return m;
}

Matrix Matrix::getMinor(int i, int j)
{
	Matrix m = Matrix(size-1);
	int q = 0;
	for (int k = 0; k < size; k++) {
		if (k != i) {
			int w = 0;
			for (int l = 0; l < size; l++) {
				if (l != j) {
					m.setValue(q, w, getValue(k, l));
					w += 1;
				}
			}
			q += 1;
		}
	}
	return m;
}

double Matrix::getDet()
{
	if (size < 2) {
		return vals[0];
	}
	else if (size == 2) {
		return (vals[0] * vals[3]) - (vals[1] * vals[2]);
	}
	else {
		double d = 0;
		double q = 1;
		for (int i = 0; i < size; i++) {
			d += q * (getValue(i, 0)*getMinor(i, 0).getDet());
			q *= -1;
		}
		return d;
	}
	return 0.0;
}

Matrix Matrix::getCofactor()
{
	Matrix m = Matrix(size);
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			double d = getMinor(i, j).getDet();
			if ((i + j) % 2)
				m.setValue(i, j, -d);
			else
				m.setValue(i, j, d);
		}
	}
	return m;
}

//Room for optimization
Matrix Matrix::getInverse()
{
	if (size <= 1) {
		Matrix m = Matrix(1);
		m.setValue(0, 0, 1 / getValue(0, 0));
		return m;
	}

	else {
		return getCofactor().getTrans() / getDet();
	}
}

Point Matrix::operator*(Point v)
{
	double x = getValue(0, 0) * v.x + getValue(0, 1) * v.y + getValue(0, 2) * v.z + getValue(0, 3);
	double y = getValue(1, 0) * v.x + getValue(1, 1) * v.y + getValue(1, 2) * v.z + getValue(1, 3);
	double z = getValue(2, 0) * v.x + getValue(2, 1) * v.y + getValue(2, 2) * v.z + getValue(2, 3);
	double w = getValue(3, 0) * v.x + getValue(3, 1) * v.y + getValue(3, 2) * v.z + getValue(3, 3);
	return Point(x/w, y/w, z/w);
}

Vector Matrix::operator*(Vector v)
{
	double x = getValue(0, 0) * v.x + getValue(0, 1) * v.y + getValue(0, 2) * v.z + getValue(0, 3);
	double y = getValue(1, 0) * v.x + getValue(1, 1) * v.y + getValue(1, 2) * v.z + getValue(1, 3);
	double z = getValue(2, 0) * v.x + getValue(2, 1) * v.y + getValue(2, 2) * v.z + getValue(2, 3);
	double w = getValue(3, 0) * v.x + getValue(3, 1) * v.y + getValue(3, 2) * v.z + getValue(3, 3);
	return Vector(x,y,z)/(w);
}

std::ostream & operator<<(std::ostream & os, Matrix const & m)
{
	for (int i = 0; i < m.size; i++) {
		for (int j = 0; j < m.size; j++) {
			os << m.vals[i*m.size + j] << " ";
		}
		os << "\n";
	}
	return os;
}
