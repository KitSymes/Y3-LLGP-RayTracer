#include "Octree.h"

Octree::Octree(Vec3f centre)
{
	_centre = centre;
	_leafObjects = std::vector<Sphere*>();
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				_children[i][j][k] = nullptr;
}

Octree::~Octree()
{
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				if (_children[i][j][k] != nullptr)
					delete _children[i][j][k];

	_leafObjects.clear();
}

void Octree::Create(std::vector<Sphere*> colliders, float sideLength)
{
	// This is a leaf because it only has 1 object
	// OR This is a leaf because the size length is <= 1
	if (colliders.size() == 1 || sideLength / 2.0f <= 2.0f)
	{
		for (Sphere* go : colliders)
			_leafObjects.push_back(go);
		return;
	}

	std::vector<Sphere*> temp[2][2][2];

	for (Sphere* go : colliders)
	{
		if (go->IntersectsLine(_centre - Vec3f(sideLength / 2.0f, 0.0f, 0.0f), _centre + Vec3f(sideLength / 2.0f, 0.0f, 0.0f)))
		{
			_leafObjects.push_back(go);
			continue;
		}
		if (go->IntersectsLine(_centre - Vec3f(0.0f, sideLength / 2.0f, 0.0f), _centre + Vec3f(0.0f, sideLength / 2.0f, 0.0f)))
		{
			_leafObjects.push_back(go);
			continue;
		}
		if (go->IntersectsLine(_centre - Vec3f(0.0f, 0.0f, sideLength / 2.0f), _centre + Vec3f(0.0f, 0.0f, sideLength / 2.0f)))
		{
			_leafObjects.push_back(go);
			continue;
		}

		Vec3f position = go->center;
		temp[(position.x > _centre.x)][(position.y > _centre.y)][(position.z > _centre.z)].push_back(go);
	}

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				if (temp[i][j][k].size() > 0)
				{
					_children[i][j][k] = new Octree(Vec3f(((i * 2) - 1), ((j * 2) - 1), ((k * 2) - 1)) * (sideLength / 2.0f) + _centre);
					_children[i][j][k]->Create(temp[i][j][k], (sideLength / 2.0f));
					temp[i][j][k].clear();
				}
}

Sphere* Octree::Trace(std::vector<Sphere*> predecessors, const Vec3f& rayorig, const Vec3f& raydir)
{
	for (Sphere* col : _leafObjects)
		predecessors.push_back(col);

	if (predecessors.size() > 1)
		for (int i = 0; i < predecessors.size(); i++)
		{
			for (int j = i + 1; j < predecessors.size(); j++)
			{
				//if (predecessors[i]->IntersectsLine(predecessors[j]))
				{
					return predecessors[i];
				}
			}
		}

	Sphere* temp = nullptr;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				if (_children[i][j][k] != nullptr)
				{
					temp = _children[i][j][k]->Trace(predecessors, rayorig, raydir);
					if (temp != nullptr)
						return temp;
				}
	return nullptr;
}
