//
// K-D Tree Spacial Data Structure
//

#pragma once
#include "Object.h"
class KDTreeNode : public Object
{
public:
	//KDTrees only store bounding box objects
	KDTreeNode *front, *rear;
	aabb vox;
	axis plane;
	double part;
	KDTreeNode(aabb vox, axis plane = xAxis, double part = 0.0, KDTreeNode *front = nullptr, KDTreeNode *rear = nullptr) : vox(vox), plane(plane), part(part), front(front), rear(rear) {}
	virtual Intersection intersect(Ray r, int excludeId = -1);
	//Internal function for recursive intersection test
	virtual Intersection _knownIntersection(Ray r, Point enter, Point leave, int excludeId = -1);
};


class KDTreeLeaf : public KDTreeNode
{
public:
	//KDTrees only store bounding box objects
	//Array of bounding box objects
	BoundingBox **contents;
	int numContents;
	KDTreeLeaf(aabb vox, BoundingBox **contents=nullptr, int numContents=0) : contents(contents), numContents(numContents), KDTreeNode(vox) {}
	//Point is assumed to intersect with the leafnode's voxel because of the recursive intersection process for the KDTree
	Intersection intersect(Ray r, int excludeId=-1);
	//Internal function for recursive intersection test
	virtual Intersection _knownIntersection(Ray r, Point enter, Point leave, int excludeId = -1);
};

//Internal use for recursive function
KDTreeNode* getNode(BoundingBox **objects, int numObjects, aabb vox, axis ax, int depth);

//Creates a KDTree from a list of objects
KDTreeNode* makeKDTree(Object **objects, int numObjects);