#include "stdafx.h"
#include "Object.h"
#include "Entities.h"


int Object::_id = 0;

Intersection Object::intersect(Ray r, int excludeId)
{
	Intersection i = _baseI();
	i.d = -1.0;
	return i;
}

aabb Object::_boundingBox()
{
	return aabb();
}

Intersection Object::_baseI()
{
	Intersection i;
	i.mat = getMaterial();
	i.id = id;
	return i;
}

Object * Object::transform(Vector translation)
{
	return new Object();
}

Intersection Sphere::intersect(Ray r, int excludeId)
{
	if (id == excludeId) {
		return NO_INTERSECTION;
	}
	double b = 2 * (r.d*Vector(p, r.o));
	double c = Vector(p, r.o).selfDot() - (rad * rad);
	double root = b * b - 4 * c;
	// No intersection
	if (root < 0) {
		return NO_INTERSECTION;
	}
	root = sqrt(root);
	double d1 = (-b + root) / 2;
	double d2 = (-b - root) / 2;

	Intersection i1 = _baseI();
	Intersection i2 = _baseI();
	i1.d = d1;
	i1.p = r.pointAt(d1);
	i1.n = Vector(p, i1.p).normalize();

	i2.d = d2;
	i2.p = r.pointAt(d2);
	i2.n = Vector(p, i2.p).normalize();

	Intersection i = leastPos(i1, i2);

	if (p.distance(r.o) < (rad+EPSILON)) {
		i.n = -1 * i.n;
		i.inside = true;
	}

	return i;
}

aabb Sphere::_boundingBox()
{
	return aabb{ p + (rad*Vector(-1,-1,-1)), p + (rad*Vector(1, 1, 1)) };
}

Object * Sphere::transform(Vector translation)
{
	
	return new Sphere(p + translation, rad, m);
}

Intersection Triangle::intersect(Ray r, int excludeId)
{
	if (id == excludeId) {
		return NO_INTERSECTION;
	}
	Vector T = Vector(a, r.o);
	Vector P = r.d.cross(e2);
	double div = P * e1;
	if (div == 0) {
		return NO_INTERSECTION;
	}
	Vector Q = T.cross(e1);

	double t = (Q*e2) / div;
	if (t <= 0) {
		return NO_INTERSECTION;
	}
	double u = (P*T) / div;
	if (u < 0) {
		return NO_INTERSECTION;
	}
	double v = (Q*r.d) / div;
	if ((v < 0) || ((u + v) > 1)) {
		return NO_INTERSECTION;
	}

	Intersection i = _baseI();
	i.d = t;
	i.p = r.pointAt(t);
	i.n = n;

	if (r.d*n > 0) {
		i.n = -1 * n;
	}

	double w = 1 - (u + v);

	i.tex.x = ta.x*w + tb.x*u + tc.x*v;
	i.tex.y = ta.y*w + tb.y*u + tc.y*v;
	i.tex.z = ta.z*w + tb.z*u + tc.z*v;

	return i;
}

aabb Triangle::_boundingBox()
{
	Point min, max;
	min.x = minimum(a.x, b.x, c.x);
	min.y = minimum(a.y, b.y, c.y);
	min.z = minimum(a.z, b.z, c.z);

	max.x = maximum(a.x, b.x, c.x);
	max.y = maximum(a.y, b.y, c.y);
	max.z = maximum(a.z, b.z, c.z);
	return aabb{ min,max };
}

Object * Triangle::transform(Vector translation)
{
	return new Triangle(a+translation, b+translation, c+translation, m);
}

Intersection Cylinder::intersect(Ray r, int excludeId)
{
	if (id == excludeId) {
		return NO_INTERSECTION;
	}
	Vector tempA = r.d - ((r.d*v)*v);
	Vector delta = Vector(a, r.o);
	Vector tempB = delta - ((delta*v)*v);

	double A = tempA.selfDot();
	double B = 2 * (tempA*tempB);
	double C = tempB.selfDot() - (rad*rad);
	
	double root = B * B - (4 * A*C);

	//No intersection
	if (root < 0) {
		return NO_INTERSECTION;
	}

	double d1 = (-B + root) / (2*A);
	double d2 = (-B - root) / (2*A);
	
	Point q1 = r.pointAt(d1);
	Point q2 = r.pointAt(d2);

	Vector tmp = Vector(a, q1);

	Intersection i1 = _baseI();;
	i1.d = d1;
	i1.p = r.pointAt(d1);
	i1.n = (tmp - ((tmp*v)*v)).normalize();

	tmp = Vector(a, q2);

	Intersection i2 = _baseI();;
	i2.d = d2;
	i2.p = r.pointAt(d2);
	i2.n = (tmp - ((tmp*v)*v)).normalize();

	if (v*Vector(a, q1) <= 0 || v*Vector(b, q1) >= 0) { //Point is above or below cylinder
		i1 = NO_INTERSECTION;
	}
	if (v*Vector(a, q2) <= 0 || v*Vector(b, q2) >= 0) { //Point is above or below cylinder
		i2 = NO_INTERSECTION;
	}

	//CAPS

	double d3 = (Vector(r.o, a)*v) / (r.d*v);
	double d4 = (Vector(r.o, b)*v) / (r.d*v);

	Point q3 = r.pointAt(d3);
	Point q4 = r.pointAt(d4);

	Intersection i3 = _baseI();;
	i3.d = d3;
	i3.p = r.pointAt(d3);
	i3.n = -1*v;

	Intersection i4 = _baseI();;
	i4.d = d4;
	i4.p = r.pointAt(d4);
	i4.n = v;

	if (Vector(q3, a).length() > rad) { //Point is outside cap
		i3 = NO_INTERSECTION;
	}
	if (Vector(q4, b).length() > rad) { //Point is outside cap
		i4 = NO_INTERSECTION;
	}

	Intersection i = leastPos(leastPos(i1, i2), leastPos(i3, i4));

	if (isPos(i) && i.n*r.d < 0) {
		i.n = -1 * i.n;
		i.inside = true;
	}

	return i;
}

//Naive interpretation, assumes sphere capped cylinder
aabb Cylinder::_boundingBox()
{
	Point min, max;
	Point q1 = a + (rad*Vector(-1, -1, -1));
	Point q2 = b + (rad*Vector(-1, -1, -1));

	Point w1 = a + (rad*Vector(1, 1, 1));
	Point w2 = b + (rad*Vector(1, 1, 1));

	min.x = minimum(q1.x, q2.x);
	min.y = minimum(q1.y, q2.y);
	min.z = minimum(q1.z, q2.z);

	max.x = maximum(w1.x, w2.x);
	max.y = maximum(w1.y, w2.y);
	max.z = maximum(w1.z, w2.z);
	
	return aabb{ min,max };
}

Object * Cylinder::transform(Vector translation)
{
	return new Cylinder(a+translation, b+translation, rad, m);
}

Intersection Box::intersect(Ray r, int excludeId)
{
	if (id == excludeId) {
		return NO_INTERSECTION;
	}

	Intersection i = _baseI();

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
		return NO_INTERSECTION;
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
		return NO_INTERSECTION;
	if (tzmin > tmin) {
		tmin = tzmin;
		nmin = nmin_t;
	}
	if (tzmax < tmax) {
		tmax = tzmax;
		nmax = nmax_t;
	}

	if (tmax <= 0)	//Entire box is behind origin
		return NO_INTERSECTION;

	if (tmin <= 0) {
		i.d = tmax;
		i.p = r.pointAt(tmax);
		i.n = nmax;
	}
	else {
		i.d = tmin;
		i.p = r.pointAt(tmin);
		i.n = nmin;
	}



	return i;
}

aabb Box::_boundingBox()
{
	return aabb{ a,b };
}

Object * Box::transform(Vector translation)
{
	return new Box(a + translation, b + translation, m);
}


BoundingBox* getBoundingBox(Object * o)
{
	aabb bound = o->_boundingBox();
	return new BoundingBox(o, bound.a, bound.b);
}

bool BoundingBox::isInside(aabb box)
{
	if (max.x < box.a.x)
		return false;
	if (min.x > box.b.x)
		return false;
	if (max.y < box.a.y)
		return false;
	if (min.y > box.b.y)
		return false;
	if (max.z < box.a.z)
		return false;
	if (min.z > box.b.z)
		return false;
	return true;
}

ObjectList::~ObjectList()
{
	for (int i = 0; i < n; i++)
		delete contents[i];
	delete[] contents;
}

Intersection ObjectList::intersect(Ray r, int excludeId)
{
	Intersection in = NO_INTERSECTION;
	for (int i = 0; i < n; i++) {
		in = leastPos(in, contents[i]->intersect(r, excludeId));
	}
	return in;
}

Object * ObjectList::transform(Vector translation)
{
	Object** newContents = new Object*[n];
	for (int i = 0; i < n; i++)
		newContents[i] = contents[i]->transform(translation);
	return new ObjectList(newContents, n);
}

Intersection GameBoard::intersect(Ray r, int excludeId)
{
	if (id == excludeId) {
		return NO_INTERSECTION;
	}

	Point enter, leave;

	double tmin, tmax, tymin, tymax, tzmin, tzmax;

	double divx = 1 / r.d.x;
	if (divx >= 0) {
		tmin = (vox.a.x - r.o.x) * divx;
		tmax = (vox.b.x - r.o.x) * divx;
	}
	else {
		tmin = (vox.b.x - r.o.x) * divx;
		tmax = (vox.a.x - r.o.x) * divx;
	}

	double divy = 1 / r.d.y;
	if (divy >= 0) {
		tymin = (vox.a.y - r.o.y) * divy;
		tymax = (vox.b.y - r.o.y) * divy;
	}
	else {
		tymin = (vox.b.y - r.o.y) * divy;
		tymax = (vox.a.y - r.o.y) * divy;
	}
	if ((tmin > tymax) || (tymin > tmax))
		return NO_INTERSECTION;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	double divz = 1 / r.d.z;
	if (divz >= 0) {
		tzmin = (vox.a.z - r.o.z) * divz;
		tzmax = (vox.b.z - r.o.z) * divz;
	}
	else {
		tzmin = (vox.b.z - r.o.z) * divz;
		tzmax = (vox.a.z - r.o.z) * divz;
	}
	if ((tmin > tzmax) || (tzmin > tmax))
		return NO_INTERSECTION;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	if (tmax <= 0)	//Entire box is behind origin
		return NO_INTERSECTION;

	if (tmin <= 0) //Point starts inside box
		tmin = 0;

	enter = r.pointAt(tmin);
	leave = r.pointAt(tmax);

	if (floor(enter.x + 2*EPSILON) != floor(enter.x)) { //Floating point error
		enter.x += 2 * EPSILON;
	}
	if (floor(enter.z + 2*EPSILON) != floor(enter.z)) { //Floating point error
		enter.z += 2 * EPSILON;
	}

	int enter_i, enter_j, leave_i, leave_j; //The boardgame coordinates of the entering and leaving squares

	enter_i = clamp((int)(floor(enter.x)),0,n-1);
	enter_j = clamp((int)(floor(enter.z)),0,n-1);

	leave_i = clamp((int)(floor(leave.x)),0,n-1);
	leave_j = clamp((int)(floor(leave.z)),0,n-1);

	//std::cout << enter_i << ", " << enter_j << "\n";
	//std::cout << leave_i << ", " << leave_j << "\n\n";

	Intersection in = NO_INTERSECTION;

	if (enter_i == leave_i) { //Light ray follows a row
		if (enter_j < leave_j) {
			for (int j = enter_j; j <= leave_j; j++) {
				in = subintersect(enter_i, j, r);
				if (isPos(in))
					return in;
			}
		}
		else {
			for (int j = enter_j; j >= leave_j; j--) {
				in = subintersect(enter_i, j, r);
				if (isPos(in))
					return in;
			}
		}
		
		return NO_INTERSECTION;
	}
	else if (enter_j == leave_j) { //Light ray follows a column
		if (enter_i < leave_i) {
			for (int i = enter_i; i <= leave_i; i++) {
				in = subintersect(i, enter_j, r);
				if (isPos(in))
					return in;
			}
		}
		else {
			for (int i = enter_i; i >= leave_i; i--) {
				in = subintersect(i, enter_j, r);
				if (isPos(in))
					return in;
			}
		}

		return NO_INTERSECTION;
	}
	else { //Light ray follows a diagonal
		/*for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				in = leastPos(in, subintersect(i, j, r));
			}
		}
		return in;*/
		int cursor_i, cursor_j;
		cursor_i = enter_i;
		cursor_j = enter_j;

		double x = enter.x;
		double z = enter.z;

		int dI = r.d.x > 0 ? 1 : -1;
		int dJ = r.d.z > 0 ? 1 : -1;

		double tI, tJ;

		bool c = true;

		while (c) {
			in = subintersect(cursor_i, cursor_j, r);
			if (isPos(in))
				return in;

			//Figure out which space towards leave_i, leave_j is closer
			if (dI > 0) { //Going +X
				tI = ((floor(x) + 1) - x) / r.d.x;
			}
			else { //Going -X
				tI = ((ceil(x) - 1) - x) / r.d.x;
			}

			if (dJ > 0) { //Going +Z
				tJ = ((floor(z) + 1) - z) / r.d.z;
			}
			else { //Going -Z
				tJ = ((ceil(z) - 1) - z) / r.d.z;
			}

			if (tI + EPSILON >= tJ && tJ + EPSILON >= tI) { //Too close to tell
				//Check both directions, then advance diagonally
				in = leastPos(subintersect(cursor_i + dI, cursor_j, r),
					subintersect(cursor_i, cursor_j + dJ, r));
				if (isPos(in))
					return in;

				cursor_i += dI;
				cursor_j += dJ;

				if (dI > 0) {
					x = floor(x) + 1;
				}
				else {
					x = ceil(x) - 1;
				}
				if (dJ > 0) {
					z = floor(z) + 1;
				}
				else {
					z = ceil(z) - 1;
				}
			}
			else if (tI < tJ) { //X direction is closer than Z
				//Advance in the X direction
				cursor_i += dI;

				if (dI > 0) {
					x = floor(x) + 1;
				}
				else {
					x = ceil(x) - 1;
				}

				z += r.d.z*tI;
			}
			else { //Z direction is closer than X
				//Advance in the Z direction
				cursor_j += dJ;

				x += r.d.x*tJ;

				if (dJ > 0) {
					z = floor(z) + 1;
				}
				else {
					z = ceil(z) - 1;
				}
			}

			if (dI > 0) {
				c = (cursor_i < leave_i);
			}
			else {
				c = (cursor_i > leave_i);
			}
			if (dJ > 0) {
				c = c || (cursor_j < leave_j);
			}
			else {
				c = c || (cursor_j > leave_j);
			}
		}

		in = subintersect(leave_i, leave_j, r);
		if (isPos(in))
			return in;
		return NO_INTERSECTION;
	}

	/*for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			in = leastPos(in, subintersect(i, j, r));
		}
	}
	return in;*/
}

Intersection GameBoard::subintersect(int i, int j, Ray r)
{
	Entity* e = b->getEntity(i, j);
	Intersection in = NO_INTERSECTION;
	if (e) {
		if (e->isHole())
		{
			in.id = i * n + j;
			return in;
		}
		else {
			Object* o = e->getModel();
			Object* t = o->transform(Vector(i + 0.5, 0, j + 0.5));
			in = t->intersect(r);
			in.id = i * n + j;
			delete t;
		}
	}

	/*Triangle* t = new Triangle(Point(i, 0, j), Point(i + 1, 0, j), Point(i, 0, j + 1), m);
	Triangle* t2 = new Triangle(Point(i+1, 0, j+1), Point(i + 1, 0, j), Point(i, 0, j + 1), m);

	in = leastPos(leastPos(t->intersect(r), t2->intersect(r)), in);

	delete t;
	delete t2;*/

	Box* b = new Box(Point(i, -0.25, j), Point(i + 1, 0, j + 1), m);
	in = leastPos(b->intersect(r), in);

	delete b;

	in.id = i * n + j;

	return in;
}

bool * GameBoard::whichIntersect(Ray r)
{
	bool* inters = new bool[n*n+1];
	for (int i = 0; i < n*n+1; i++) {
		inters[i] = false;
	}

	Point a = Point(n/2, -2.5, n/2);
	Point b = Point((n/2)+1, 1, (n/2)+1);
	if (doesIntersect(r, a, b))
		inters[n*n] = true;

	Point enter, leave;

	double tmin, tmax, tymin, tymax, tzmin, tzmax;

	double divx = 1 / r.d.x;
	if (divx >= 0) {
		tmin = (vox.a.x - r.o.x) * divx;
		tmax = (vox.b.x - r.o.x) * divx;
	}
	else {
		tmin = (vox.b.x - r.o.x) * divx;
		tmax = (vox.a.x - r.o.x) * divx;
	}

	double divy = 1 / r.d.y;
	if (divy >= 0) {
		tymin = (vox.a.y - r.o.y) * divy;
		tymax = (vox.b.y - r.o.y) * divy;
	}
	else {
		tymin = (vox.b.y - r.o.y) * divy;
		tymax = (vox.a.y - r.o.y) * divy;
	}
	if ((tmin > tymax) || (tymin > tmax))
		return inters; //No intersections
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	double divz = 1 / r.d.z;
	if (divz >= 0) {
		tzmin = (vox.a.z - r.o.z) * divz;
		tzmax = (vox.b.z - r.o.z) * divz;
	}
	else {
		tzmin = (vox.b.z - r.o.z) * divz;
		tzmax = (vox.a.z - r.o.z) * divz;
	}
	if ((tmin > tzmax) || (tzmin > tmax))
		return inters; //No intersection
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	if (tmax <= 0)	//Entire box is behind origin
		return inters;

	if (tmin <= 0) //Point starts inside box
		tmin = 0;

	enter = r.pointAt(tmin);
	leave = r.pointAt(tmax);

	if (floor(enter.x + 2 * EPSILON) != floor(enter.x)) { //Floating point error
		enter.x += 2 * EPSILON;
	}
	if (floor(enter.z + 2 * EPSILON) != floor(enter.z)) { //Floating point error
		enter.z += 2 * EPSILON;
	}

	int enter_i, enter_j, leave_i, leave_j; //The boardgame coordinates of the entering and leaving squares

	enter_i = clamp((int)(floor(enter.x)), 0, n - 1);
	enter_j = clamp((int)(floor(enter.z)), 0, n - 1);

	leave_i = clamp((int)(floor(leave.x)), 0, n - 1);
	leave_j = clamp((int)(floor(leave.z)), 0, n - 1);

	Intersection in = NO_INTERSECTION;

	if (enter_i == leave_i) { //Light ray follows a row
		if (enter_j < leave_j) {
			for (int j = enter_j; j <= leave_j; j++) {
				inters[enter_i*n + j] = true;
			}
		}
		else {
			for (int j = enter_j; j >= leave_j; j--) {
				inters[enter_i*n + j] = true;
			}
		}
	}
	else if (enter_j == leave_j) { //Light ray follows a column
		if (enter_i < leave_i) {
			for (int i = enter_i; i <= leave_i; i++) {
				inters[i*n + enter_j] = true;
			}
		}
		else {
			for (int i = enter_i; i >= leave_i; i--) {
				inters[i*n + enter_j] = true;
			}
		}
	}
	else { //Light ray follows a diagonal

		for (int i = minimum(enter_i, leave_i); i <= maximum(enter_i, leave_i); i++) {
			for (int j = minimum(enter_j, leave_j); j <= maximum(enter_j, leave_j); j++) {
				a = Point(i, -0.25, j);
				b = Point(i + 1, 1, j + 1);
				if (doesIntersect(r, a, b))
					inters[i*n + j] = true;
			}
		}
	}

	return inters;
}
