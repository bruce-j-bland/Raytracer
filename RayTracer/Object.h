//
// Raytracer Objects
//

#pragma once

#include "Util.h"
#include "Board.h"

class Object
{
private:
	static int _id;
public:
	int id;
	bool isBB = false;
	Material* m;
	Object(Material* m = nullptr) : m(m), id(_id++) {};
	virtual Material* getMaterial() { return m; }
	//Returns the distance to intersection, or a negative number if no intersection exists
	virtual Intersection intersect(Ray r, int excludeId=-1);
	//Helper function that returns coordinates for axis-aligned bounding box
	//Use getBoundingBox(Object* o) to create bounding box
	virtual aabb _boundingBox();
	//Used whenever returning an intersection
	Intersection _baseI();
	//Used when generating an object from a model
	virtual Object* transform(Vector translation);
};


//Surrounds single objects with a bounding box to make it easier to calculate
//For k-d trees
class BoundingBox : public Object
{
public:
	bool isBB = true;
	Object* o;
	Point min, max;
	BoundingBox(Object* contents, Point min, Point max) : o(contents), min(min), max(max) {}
	Material* getMaterial() { return o->getMaterial(); }
	Intersection intersect(Ray r, int excludeId = -1) { return o->intersect(r, excludeId); }
	//Why would you need a bounding box for a single bounding box?
	aabb _boundingBox() { return aabb{ min,max }; }
	//Returns true if the object is at least partially inside the axis aligned bounding box
	bool isInside(aabb box);
};

class ObjectList : public Object
{
public:
	Object** contents;
	int n = 0;
	ObjectList(Object** contents, int n) : contents(contents), n(n) {}
	~ObjectList();
	Intersection intersect(Ray r, int excludeId = -1);
	Object* transform(Vector translation);
};

class Sphere : public Object
{
public:
	Point p;
	double rad;
	Sphere(Point center, double radius, Material* m) : p(center), rad(radius), Object(m) {};
	Intersection intersect(Ray r, int excludeId = -1);
	aabb _boundingBox();
	Object* transform(Vector translation);
};

//Counterclockwise order
class Triangle : public Object
{
public:
	Point a, b, c;
	Vector e1, e2, n;
	Triangle(Point a, Point b, Point c, Material* m) : a(a), b(b), c(c), Object(m) {
		e1 = Vector(a, b);
		e2 = Vector(a, c);
		n = e1.cross(e2).normalize();
	};
	Intersection intersect(Ray r, int excludeId = -1);
	aabb _boundingBox();
	Object* transform(Vector translation);
};

class Cylinder : public Object
{
public:
	Point a, b;
	Vector v;
	double rad;
	Cylinder(Point a, Point b, double rad, Material* m) : a(a), b(b), rad(rad), Object(m) {
		v = Vector(a, b).normalize();
	}
	Intersection intersect(Ray r, int excludeId = -1);
	aabb _boundingBox();
	Object* transform(Vector translation);
};

class Box : public Object
{
public:
	Point a, b;
	Box(Point alpha, Point beta, Material* m) : Object(m) {
		a = minimum(alpha, beta);
		b = maximum(alpha, beta);
	}
	Intersection intersect(Ray r, int excludeId = -1);
	aabb _boundingBox();
	Object* transform(Vector translation);
};

class GameBoard : public Object
{
public:
	Board * b;
	int n;
	aabb vox;
	GameBoard(Board* b, int n, Material* m) : b(b), n(n), Object(m) { vox.a = Point(0, -.25, 0); vox.b = Point(n, 1, n); }
	Intersection intersect(Ray r, int excludeId = -1);
	Intersection subintersect(int i, int j, Ray r);
};

BoundingBox* getBoundingBox(Object* o);
