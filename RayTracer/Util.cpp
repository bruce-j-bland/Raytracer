#include "stdafx.h"
#include "Util.h"


Point::Point(const Point & p)
{
	x = p.x;
	y = p.y;
	z = p.z;
}

double Point::distance(Point p)
{
	double dx = p.x - x;
	double dy = p.y - y;
	double dz = p.z - z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

Point Point::transform(double dx, double dy, double dz)
{
	return Point(x + dx, y + dy, z + dz);
}

Vector reflect(Vector incident, Vector normal)
{
	return incident - (2*normal*(incident*normal));
}

Point operator+(Point p, Vector v) {
	return Point(p.x + v.x, p.y + v.y, p.z + v.z);
}

Color operator*(double d, Color c)
{
	return Color(d*c.r, d*c.g, d*c.b);
}

Color average(Color * colors, int num)
{
	double r = 0.0;
	double g = 0.0;
	double b = 0.0;
	for (int i = 0; i < num; i++) {
		r += colors[i].r;
		g += colors[i].g;
		b += colors[i].b;
	}
	return Color(r/num, g/num, b/num);
}

Vector::Vector(Point a, Point b)
{
	x = b.x - a.x;
	y = b.y - a.y;
	z = b.z - a.z;
}

Vector::Vector(const Vector &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

Vector Vector::operator+(Vector v)
{
	return Vector(x+v.x, y+v.y, z+v.z);
}

Vector Vector::operator-(Vector v)
{
	return Vector(x - v.x, y - v.y, z - v.z);
}

Vector Vector::operator*(double d)
{
	return Vector(x*d, y*d, z*d);
}

Vector Vector::operator/(double d)
{
	return Vector(x/d, y/d, z/d);
}

double Vector::operator*(Vector v)
{
	return x*v.x + y*v.y + z*v.z;
}

Vector Vector::cross(Vector v)
{
	return Vector((y*v.z)-(z*v.y),(z*v.x)-(x*v.z),(x*v.y)-(y*v.x));
}

double Vector::selfDot()
{
	return x * x + y * y + z * z;
}

double Vector::length()
{
	return sqrt(x*x + y*y + z*z);
}

Vector Vector::normalize()
{
	return Vector(x,y,z)/length();
}

std::ostream & operator<<(std::ostream & os, Point const & p)
{
	return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
}

Vector operator*(double s, Vector v)
{
	return Vector(s*v.x, s*v.y, s*v.z);
}

std::ostream & operator<<(std::ostream & os, Vector const & v)
{
	return os << "<" << v.x << ", " << v.y << ", " << v.z << ">";
}

void Image::setPixel(int x, int y, Color c, int id)
{
	mu.lock();
	pixels[x + y * width] = c;
	ids[x + y * width] = id;
	double m = maximum(c.r, c.g, c.b);
	maxIntensity = m > maxIntensity ? m : maxIntensity;
	mu.unlock();
}

Color Image::getPixel(int x, int y)
{
	return pixels[x + y * width] * (1/maxIntensity);
}

int Image::getId(int x, int y)
{
	return ids[x + y * width];
}

//Compare against all adjacent and orthogonal pixels
//If any are from different objects, return true
bool Image::isEdge(int x, int y)
{
	int c = ids[x + y * width];

	if (x > 0) {
		if(c != ids[x - 1 + y * width])
			return true;
		if (y > 0 && c != ids[x - 1 + (y - 1) * width])
			return true;
		if (y < height - 1 && c != ids[x - 1 + (y + 1) * width])
			return true;
	}
	if (y > 0 && c != ids[x + (y - 1) * width])
		return true;
	if (y < height - 1 && c != ids[x + (y + 1) * width])
		return true;
	if (x < width-1) {
		if (c != ids[x + 1 + y * width])
			return true;
		if (y > 0 && c != ids[x + 1 + (y - 1) * width])
			return true;
		if (y < height - 1 && c != ids[x + 1 + (y + 1) * width])
			return true;
	}

	return false;
}

Point Ray::pointAt(double dist)
{
	return o + (d*dist);
}

double Ray::distTo(double q, axis ax)
{
	switch (ax) {
		case(xAxis):
			if (o.x == q)
				return 0;
			if (d.x == 0)
				return -1;
			return (q - o.x) / d.x;
		case(yAxis):
			if (o.y == q)
				return 0;
			if (d.y == 0)
				return -1;
			return (q - o.y) / d.y;
		case(zAxis):
			if (o.z == q)
				return 0;
			if (d.z == 0)
				return -1;
			return (q - o.z) / d.z;
	}
	return -1.0;
}

Color Color::operator+(Color c)
{
	return Color(r+c.r, g+c.g, b+c.b);
}

Color Color::operator*(double d)
{
	return Color(r*d, g*d, b*d);
}

Color Color::operator*(Color c)
{
	return Color(r * c.r, g * c.g, b * c.b);
}

Color PhongMaterial::getColor(Intersection in, Light ** lights, int numLights, Color ambientLight)
{
	Color amb = base * ambientLight;

	Color dif = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Color q = (base*lights[i]->c)*(Vector(in.p, lights[i]->p).normalize()*in.n);
		if (q.r >= 0 && q.g >= 0 && q.b >= 0)
			dif = dif + q;
	}

	Color spec = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Vector r = reflect(Vector(lights[i]->p, in.p).normalize(), in.n);
		double factor = (r*in.eye);
		if(factor > 0)	//Avoid specular edge with even exponents
			spec = spec + (specular*lights[i]->c)*(pow(factor, ke));
	}

	return (ka * amb) + (kd * dif) + (ks * spec);
}

Color BlinnMaterial::getColor(Intersection in, Light ** lights, int numLights, Color ambientLight)
{
	Color amb = base * ambientLight;

	Color dif = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Color q = (base*lights[i]->c)*(Vector(in.p, lights[i]->p).normalize()*in.n);
		if (q.r >= 0 && q.g >= 0 && q.b >= 0)
			dif = dif + q;
	}

	Color spec = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Vector h = (Vector(in.p, lights[i]->p).normalize() + in.eye).normalize();
		Color q = (specular*lights[i]->c)*(pow((h*in.n), ke));
		if (q.r >= 0 && q.g >= 0 && q.b >= 0)
			spec = spec + q;
	}

	return (ka * amb) + (kd * dif) + (ks * spec);
}
