//
// Utility classes and objects
// Custom Geometry Objects
//

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <mutex>

#pragma once

#define EPSILON 0.001

//The axises

enum axis { xAxis, yAxis, zAxis };

//A point in 3d space
class Point
{
public:
	double x, y, z;
	Point(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
	//Copy constructor
	Point(const Point &p);
	//Euclidian Distance between two points
	double distance(Point p);
	//Return a translated point
	Point transform(double dx, double dy, double dz);
};

//Returns whether a <= b for every dimension
inline bool strictlyLess(Point a, Point b) {
	if (a.x > b.x || a.y > b.y || a.z > b.z)
		return false;
	return true;
}

//Used for axis aligned bounding boxes
typedef struct aabb {
	Point a;
	Point b;
} aabb;

const aabb UNSET_AABB = { Point(1000,1000), Point(-1000,-1000) };

//String representation
std::ostream &operator<<(std::ostream &os, Point const &p);

//A vector in 3d space
class Vector
{
public:
	double x, y, z;
	Vector(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
	//Define Vector with two points, from a to b
	Vector(Point a, Point b);
	//Copy constructor
	Vector(const Vector &v);
	//Simple vector arithmetic
	Vector operator+(Vector v);
	Vector operator-(Vector v);
	//Scalar reverse multiplication
	Vector operator*(double d);
	//Scalar division
	Vector operator/(double d);
	//Dot product
	double operator*(Vector v);
	//Cross product
	Vector cross(Vector v);
	//Dotting with itself. Length^2
	double selfDot();
	//Length of the vector
	double length();
	//Returns the normalized vector
	Vector normalize();
};

//Scalar multiplication
Vector operator*(double s, Vector v);
//String representation
std::ostream &operator<<(std::ostream &os, Vector const &v);

//Incoming incident vector hits a point with a normal and reflects out perfectly
//Normal is assumed normalized
Vector reflect(Vector incident, Vector normal);

//Adding a vector to a point
Point operator+(Point p, Vector v);

//Vector's from a point source
//The vector is always normalized
class Ray
{
public:
	Point o;
	Vector d;
	Ray(Point origin, Point p) : o(origin), d(Vector(origin, p).normalize()) {}
	Ray(Point origin, Vector direction) : o(origin), d(direction.normalize()) {}
	//Get point dist distance along the ray
	Point pointAt(double dist);
	//Get the distance on which the ray's ax axis value is equal to q, e.g. r.pointAt(3,xAxis) returns the distance on the ray to make x = 3
	//Returns -1 if impossible to reach
	double distTo(double q, axis ax);
};

//RGB 0.0-1.0 colors
//Also represents radiosity
class Color
{
public:
	double r, g, b;
	Color(double r=0.0, double g=0.0, double b=0.0) : r(r), g(g), b(b) {}
	Color operator+(Color c);
	Color operator*(double d);
	Color operator*(Color c);
	double getIlluminance();
};
Color operator*(double d, Color c);

const Color WHITE = Color(1.0, 1.0, 1.0);
const Color BLACK = Color(0, 0, 0);
const Color RED = Color(1.0, 0, 0);
const Color GREEN = Color(0, 1.0, 0);
const Color BLUE = Color(0, 0, 1.0);

//Returns the average of colors
Color average(Color* colors, int num);

//A radiosity in a position
class Light
{
public:
	Color c; //Radiosity
	Point p; //Position
	Light(Color c, Point p) : c(c), p(p) {}
};

class Material; //Forward declaration

typedef struct Intersection {
	double d; //Distance along ray
	Point p; //Point of intersection
	Vector n; //Normal vector (normalized)
	Material* mat; //Material/illumination model of object
	int id;	//Object id
	Vector eye; //Vector from point to eye, used by Phong, normalized
	Point tex; //Point for texture mapping
	bool inside = false;
} Intersection;

//Material properties for objects
class Material
{
	double r = 0.0; //Reflectivity
	double t = 0.0; //Transparency
	double n = 1.0; //Index of Refraction
public:
	Material() { }
	void setReflectivity(double rv) { r = rv; };
	void setTransparency(double tv, double nv = 1.0) { t = tv; n = nv; };
	virtual double getReflectivity(struct Intersection in) { return r; };
	virtual double getTransparency(struct Intersection in) { return t; };
	virtual double getIndexOfRefraction(struct Intersection in) { return n; };
	virtual Color getColor(struct Intersection in, Light** lights, int numLights, Color ambient) { return Color(0, 0, 0); };
};

//Object is single color, no illumination model
class FlatMaterial : public Material
{
public:
	Color c;
	FlatMaterial(Color c = BLACK) : c(c) {};
	Color getColor(struct Intersection in, Light** lights, int numLights, Color ambientLight) { return c; }
};

//Phong Illumination model
class PhongMaterial : public Material
{
public:
	Color base, specular;
	double ka, kd, ks, ke;
	PhongMaterial(Color base, Color specular=WHITE,
		double ka=0.2, double kd=0.4, double ks=0.1, double ke=4)
		: base(base), specular(specular),
		ka(ka), kd(kd), ks(ks), ke(ke) {};
	Color getColor(struct Intersection in, Light** lights, int numLights, Color ambientLight);
};

//Phong-Blinn Illumination model
class BlinnMaterial : public Material
{
public:
	Color base, specular;
	double ka, kd, ks, ke;
	BlinnMaterial(Color base, Color specular = WHITE,
		double ka = 0.2, double kd = 0.4, double ks = 0.1, double ke = 4)
		: base(base), specular(specular),
		ka(ka), kd(kd), ks(ks), ke(ke) {};
	Color getColor(struct Intersection in, Light** lights, int numLights, Color ambientLight);
};

//An n by n Checkerboard Pattern of materials a and b
//Uses texture coordinates
class Checkerboard : public Material
{
	Material *a, *b;
	int n;
public:
	Checkerboard(Material* a, Material* b, int n) : a(a), b(b), n(n) {};
	double getReflectivity(struct Intersection in);
	virtual double getTransparency(struct Intersection in);
	virtual double getIndexOfRefraction(struct Intersection in);
	Color getColor(struct Intersection in, Light** lights, int numLights, Color ambientLight);
};

const Intersection NO_INTERSECTION = { -1, Point(), Vector(), new FlatMaterial(BLACK), -1 };


class Image; //Forward declaration

//Uses phong as a base
class ImageTexture : public Material
{
	Image* im;
public:
	ImageTexture(Image* im) : im(im) {};
	Color getColor(struct Intersection in, Light** lights, int numLights, Color ambientLight);
};


//Holds a 2d array of pixels
class Image
{
public:
	int width, height;
	Color* pixels;
	int* ids;
	double maxIntensity = 0.01;
	double ldMax = 200;
	std::mutex mu; //Protect modification to make threadsafe
	Image(int width, int height) : width(width), height(height) { pixels = new Color[width*height]; ids = new int[width*height]; }
	~Image() { delete[] pixels; delete[] ids; };
	void setLDMax(double nits) { ldMax = nits; }
	void scale(double sf); //Scales all pixels by sf;
	void naiveScale(); //Call after setting all pixels before rendering them if not doing tone reproduction
	void wardTone(); //Ward tone reproduction
	void photoTone(); //Reinhard tone reporduction; Uses log average luminance as key to map to middle grey
	void photoTone(double key); //Renhard tone reproduction
	void photoTone(int x, int y); //Reinhard tone reproduction; Uses pixel at x y for key to map to middle grey
	void histogramAdjustment(bool applyCeiling = true); //Histogram tone reproduction, with and without clamping
	void deviceScale(); //Scales by 1/ldMax; Call after tone reproduction before rendering

	void setPixel(int x, int y, Color c, int id=0);
	Color getPixel(int x, int y);
	int getId(int x, int y);
	//Returns if pixel represent the edge of an object
	bool isEdge(int x, int y);

	//Returns log average luminance
	double getLogAverageLuminance();
};

//PixelMask is used to only update part of an existing image
//It stores a number of different masks in the maskSet and selectively chooses between them
class PixelMask {
private:
	bool** maskSet;
	bool* mask;
	std::mutex mu; //Protect modification to make threadsafe
public:
	int width, height, n;

	PixelMask(int width, int height, int n) : width(width), height(height), n(n) {
		mask = new bool[width*height];
		maskSet = new bool*[width*height];
		for (int i = 0; i < width*height; i++) {
			mask[i] = false;
		}
	}
	~PixelMask() {
		for (int i = 0; i < width*height; i++) {
			delete[] maskSet[i];
		}
		delete[] maskSet;
		delete[] mask;
	}

	void setPixelMask(int x, int y, bool* m); //Set all masks for a pixel
	void loadMask(int id); //Activate a mask
	void resetMasks(); //Deactivate all masks
	bool getPixelMask(int x, int y); //Return whether the pixel is on in at least one active mask
};

//Stores info on color and which object was hit
typedef struct CO {
	Color c;
	int id;
} CO;

//Returns the least positive value, or -1, if both are nonpositive
//Zero is not positive
inline double leastPos(double x, double y)
{
	if (x <= 0) {
		if (y <= 0) {
			return -1.0;
		}
		else {
			return y;
		}
	}
	else {
		if (y <= 0 || x<=y) {
			return x;
		}
		else {
			return y;
		}
	}
}

inline bool isPos(Intersection x) {
	return x.d > EPSILON;
}

//Returns the least positive intersection object, or NO_INTERSECTION, if both are nonpositive
//Zero is not positive
inline Intersection leastPos(Intersection x, Intersection y)
{
	if (x.d <= EPSILON) {
		if (y.d <= EPSILON) {
			return NO_INTERSECTION;
		}
		else {
			return y;
		}
	}
	else {
		if (y.d <= 0 || x.d <= y.d) {
			return x;
		}
		else {
			return y;
		}
	}
}

//Minimum of 2 or 3 values
inline double minimum(double x, double y, double z = 100000)
{
	if (x <= y) {
		if (x <= z) {
			return x;
		}
		else {
			return z;
		}
	}
	else if (y <= z) {
		return y;
	}
	else {
		return z;
	}
}

//Maximum of 2 or 3 values
inline double maximum(double x, double y, double z = -100000)
{
	if (x >= y) {
		if (x >= z) {
			return x;
		}
		else {
			return z;
		}
	}
	else if (y >= z) {
		return y;
	}
	else {
		return z;
	}
}

//Constructs a point with the minimum x,y,z values of a and b
inline Point minimum(Point a, Point b) {
	Point m;
	m.x = a.x < b.x ? a.x : b.x;
	m.y = a.y < b.y ? a.y : b.y;
	m.z = a.z < b.z ? a.z : b.z;
	return m;
}

//Constructs a point with the maximum x,y,z values of a and b
inline Point maximum(Point a, Point b) {
	Point m;
	m.x = a.x > b.x ? a.x : b.x;
	m.y = a.y > b.y ? a.y : b.y;
	m.z = a.z > b.z ? a.z : b.z;
	return m;
}

template<class T>
inline T clamp(T v, T min, T max) {
	if (v < min)
		return min;
	if (v > max)
		return max;
	return v;
}


//Returns true if a ray intersects with an axis aligned box formed from two points
inline bool doesIntersect(Ray r, Point a, Point b) {

	double tmin, tmax, tymin, tymax, tzmin, tzmax;
	Vector nmin, nmax, nmin_t, nmax_t;

	double divx = 1 / r.d.x;
	if (divx >= 0) {
		tmin = (a.x - r.o.x) * divx;
		tmax = (b.x - r.o.x) * divx;
		nmin = Vector(-1, 0, 0);
		nmax = Vector(1, 0, 0);
	}
	else {
		tmin = (b.x - r.o.x) * divx;
		tmax = (a.x - r.o.x) * divx;
		nmin = Vector(1, 0, 0);
		nmax = Vector(-1, 0, 0);
	}

	double divy = 1 / r.d.y;
	if (divy >= 0) {
		tymin = (a.y - r.o.y) * divy;
		tymax = (b.y - r.o.y) * divy;
		nmin_t = Vector(0, -1, 0);
		nmax_t = Vector(0, 1, 0);
	}
	else {
		tymin = (b.y - r.o.y) * divy;
		tymax = (a.y - r.o.y) * divy;
		nmin_t = Vector(0, 1, 0);
		nmax_t = Vector(0, -1, 0);
	}
	if ((tmin > tymax) || (tymin > tmax))
		return false;
	if (tymin > tmin) {
		tmin = tymin;
		nmin = nmin_t;
	}
	if (tymax < tmax) {
		tmax = tymax;
		nmax = nmax_t;
	}

	double divz = 1 / r.d.z;
	if (divz >= 0) {
		tzmin = (a.z - r.o.z) * divz;
		tzmax = (b.z - r.o.z) * divz;
		nmin_t = Vector(0, 0, -1);
		nmax_t = Vector(0, 0, 1);
	}
	else {
		tzmin = (b.z - r.o.z) * divz;
		tzmax = (a.z - r.o.z) * divz;
		nmin_t = Vector(0, 0, 1);
		nmax_t = Vector(0, 0, -1);
	}
	if ((tmin > tzmax) || (tzmin > tmax))
		return false;
	if (tzmin > tmin) {
		tmin = tzmin;
		nmin = nmin_t;
	}
	if (tzmax < tmax) {
		tmax = tzmax;
		nmax = nmax_t;
	}

	if (tmax <= 0)	//Entire box is behind origin
		return false;

	return true;
}