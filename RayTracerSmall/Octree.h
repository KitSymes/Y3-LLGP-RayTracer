#pragma once
#ifndef _OCTREE_H
#define _OCTREE_H

#include <vector>
#include "Vec3.h"
#include "Sphere.h"

typedef Vec3<float> Vec3f;
class Octree
{
private:
	Octree* _children[2][2][2];
	std::vector<Sphere*> _leafObjects;
	Vec3f _centre;
public:
	Octree(Vec3f centre);
	~Octree();

	void Create(std::vector<Sphere*> colliders, float sideLength);
	Sphere* Trace(std::vector<Sphere*> predecessors, const Vec3f& rayorig, const Vec3f& raydir);
};
#endif
