#include "stdafx.h"
#include "KDTree.h"

Intersection KDTreeLeaf::intersect(Ray r, int excludeId)
{
	Intersection inter = NO_INTERSECTION;
	for (int i = 0; i < numContents; i++) {
		inter = leastPos(inter, contents[i]->intersect(r, excludeId));
	}
	return inter;
}

Intersection KDTreeLeaf::_knownIntersection(Ray r, Point enter, Point leave, int excludeId)
{
	return intersect(r, excludeId);
}

KDTreeNode * getNode(BoundingBox ** objects, int numObjects, aabb vox, axis ax, int depth)
{
	if (numObjects < 2 || depth > 20) {
		//For memory efficiency
		if (numObjects == 0) {
			return new KDTreeLeaf(vox);
		}
		else {
			BoundingBox **contents = new BoundingBox*[numObjects];
			for (int i = 0; i < numObjects; i++)
				contents[i] = objects[i];
			return new KDTreeLeaf(vox, contents, numObjects);
		}
	}

	axis nextAxis;
	double part;

	aabb frontVox;
	BoundingBox **frontObjects = new BoundingBox*[numObjects];
	int numFrontObjects = 0;

	aabb rearVox;
	BoundingBox **rearObjects = new BoundingBox*[numObjects];
	int numRearObjects = 0;

	switch (ax) {
		case xAxis:
			nextAxis = yAxis;
			part = vox.a.x + vox.b.x;
			part /= 2;

			frontVox.a = vox.a;
			frontVox.b = Point(part, vox.b.y, vox.b.z);
			rearVox.a = Point(part, vox.a.y, vox.a.z);
			rearVox.b = vox.b;

			break;
		case yAxis:
			nextAxis = zAxis;
			part = vox.a.y + vox.b.y;
			part /= 2;

			frontVox.a = vox.a;
			frontVox.b = Point(vox.b.x, part, vox.b.z);
			rearVox.a = Point(vox.a.x, part, vox.a.z);
			rearVox.b = vox.b;

			break;
		case zAxis:
			nextAxis = xAxis;
			part = vox.a.z + vox.b.z;
			part /= 2;

			frontVox.a = vox.a;
			frontVox.b = Point(vox.b.x, vox.b.y, part);
			rearVox.a = Point(vox.a.x, vox.a.y, part);
			rearVox.b = vox.b;
			break;
	}

	for (int i = 0; i < numObjects; i++) {
		if (objects[i]->isInside(frontVox)) {
			frontObjects[numFrontObjects++] = objects[i];
		}
		if (objects[i]->isInside(rearVox)) {
			rearObjects[numRearObjects++] = objects[i];
		}
	}

	return new KDTreeNode(vox, ax, part, 
		getNode(frontObjects, numFrontObjects, frontVox, nextAxis, depth+1),
		getNode(rearObjects, numRearObjects, rearVox, nextAxis, depth+1));
}

KDTreeNode * makeKDTree(Object ** objects, int numObjects)
{
	BoundingBox **boxes = new BoundingBox*[numObjects];
	aabb vox = UNSET_AABB;

	for (int i = 0; i < numObjects; i++) {
		if (objects[i]->isBB) {
			boxes[i] = (BoundingBox *)objects[i];
		}
		else {
			boxes[i] = getBoundingBox(objects[i]);
		}
		vox.a = minimum(vox.a, boxes[i]->min);
		vox.b = maximum(vox.b, boxes[i]->max);
	}

	return getNode(boxes, numObjects, vox, xAxis, 0);
}

Intersection KDTreeNode::intersect(Ray r, int excludeId)
{
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

	enter = r.pointAt(tmin);
	leave = r.pointAt(tmax);

	return _knownIntersection(r, enter, leave, excludeId);
}

//The enter and leave values are precomputed at this point
Intersection KDTreeNode::_knownIntersection(Ray r, Point enter, Point leave, int excludeId)
{
	double s, a, b;
	s = part;
	switch(plane) {
		case xAxis:
			a = enter.x;
			b = leave.x;
			break;
		case yAxis:
			a = enter.y;
			b = leave.y;
			break;
		case zAxis:
			a = enter.z;
			b = leave.z;
			break;
	}

	if (a <= s) {
		if (b <= s) {	//Only passes through front
			return front->_knownIntersection(r, enter, leave, excludeId);
		}
		else { //Passes through front, then passes through rear
			double d = r.distTo(s, plane);
			Point midPoint = r.pointAt(d); //Point passes through the intersection plane
			if (d <= 0) { //intersection plane is behind ray origin
				return rear->_knownIntersection(r, midPoint, leave, excludeId);
			}
			else { //intersection plane is in front of ray origin
				Intersection i = front->_knownIntersection(r, enter, midPoint, excludeId);
				if (isPos(i))
					return i;
				return rear->_knownIntersection(r, midPoint, leave, excludeId);
			}
		}
	}
	else {
		if (b > s) {	//Only passes through rear
			return rear->_knownIntersection(r, enter, leave, excludeId);
		}
		else { //Passes through rear, then passes through front
			double d = r.distTo(s, plane);
			Point midPoint = r.pointAt(d); //Point passes through the intersection plane
			if (d <= 0) { //intersection plane is behind ray origin
				return front->_knownIntersection(r, midPoint, leave, excludeId);
			}
			else { //intersection plane is in front of ray origin
				Intersection i = rear->_knownIntersection(r, enter, midPoint, excludeId);
				if (isPos(i))
					return i;
				return front->_knownIntersection(r, midPoint, leave, excludeId);
			}
		}
	}

	return Intersection();
}
