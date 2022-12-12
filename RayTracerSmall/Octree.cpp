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

void Octree::Create(std::vector<Sphere*> spheres, float sideLength)
{
	this->sideLength = sideLength;
	Vec3f min = _centre - sideLength;
	Vec3f max = _centre + sideLength;

	xmin = min.x;
	ymin = min.y;
	zmin = min.z;
	xmax = max.x;
	ymax = max.y;
	zmax = max.z;

	// This is a leaf because it only has 1 object
	// OR This is a leaf because the size length is <= 1
	if (spheres.size() <= 1 || sideLength / 2.0f <= 2.0f)
	{
		for (Sphere* go : spheres)
			_leafObjects.push_back(go);
		return;
	}

	std::vector<Sphere*> temp[2][2][2];

	for (Sphere* go : spheres)
	{
		if (go->IntersectsPlane(_centre, Vec3f(1.0f, 0.0f, 0.0f)))
		{
			_leafObjects.push_back(go);
			continue;
		}
		if (go->IntersectsPlane(_centre, Vec3f(0.0f, 1.0f, 0.0f)))
		{
			_leafObjects.push_back(go);
			continue;
		}
		if (go->IntersectsPlane(_centre, Vec3f(0.0f, 0.0f, 1.0f)))
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

Sphere* Octree::Trace(const Vec3f& rayOrig, const Vec3f& rayDir, float tx0, float ty0, float tz0, float tx1, float ty1, float tz1, unsigned char xor, float& tnear)
{
	Sphere* sphere = nullptr;

	if (_leafObjects.size() > 0)
	{
		//Sphere* temp = nullptr;
		float t0 = INFINITY, t1 = INFINITY;
		for (unsigned i = 0; i < _leafObjects.size(); ++i) {
			if (_leafObjects[i]->intersect(rayOrig, rayDir, t0, t1)) {
				if (t0 < 0) t0 = t1;
				if (t0 < tnear) {
					tnear = t0;
					sphere = _leafObjects[i];
				}
			}
		}
	}

	if (tx1 < 0.0 || ty1 < 0.0 || tz1 < 0.0)
		return sphere;

	/*unsigned char a = 0;
	Vec3f dir = rayDir;
	Vec3f orig = rayOrig;

	if (rayDir.x < 0.0)
	{
		orig.x = sideLength - rayOrig.x;
		dir.x = -rayDir.x;
		a |= 4;
	}
	if (rayDir.y < 0.0)
	{
		orig.y = sideLength - rayOrig.y;
		dir.y = -rayDir.y;
		a |= 2;
	}
	if (rayDir.z < 0.0)
	{
		orig.z = sideLength - rayOrig.z;
		dir.z = -rayDir.z;
		a |= 1;
	}

	float tx0 = (xmin - orig.x) / dir.x;
	float tx1 = (xmax - orig.x) / dir.x;
	float ty0 = (ymin - orig.y) / dir.y;
	float ty1 = (ymax - orig.y) / dir.y;
	float tz0 = (zmin - orig.z) / dir.z;
	float tz1 = (zmax - orig.z) / dir.z;*/

	float txm = 0.5 * (tx0 + tx1);
	float tym = 0.5 * (ty0 + ty1);
	float tzm = 0.5 * (tz0 + tz1);

	int node = 0;

	bool zero = false;
	bool one = false;
	bool two = false;

	if (tx0 > ty0 && tx0 > tz0) // Entry plane is YZ
	{
		if (tym < tx0)
			one = true;
		if (tzm < tx0)
			two = true;
	}
	else if (ty0 > tz0) // Entry plane is XZ
	{
		if (txm < ty0)
			zero = true;
		if (tzm < ty0)
			two = true;
	}
	else // Entry plane is XY
	{
		if (txm < tz0)
			zero = true;
		if (tym < tz0)
			one = true;
	}

	if (zero)
		node |= 4;
	if (one)
		node |= 2;
	if (two)
		node |= 1;

	Octree* next;
	Sphere* temp;
	while (node < 8)
	{
		switch (node)
		{
		case 0: // MIN X Y Z
			next = childFromLabel(node ^ xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, tx0, ty0, tz0, txm, tym, tzm, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(txm, 4, tym, 2, tzm, 1);
			break;
		case 1: // MIN X Y MAX Z
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, tx0, ty0, tzm, txm, tym, tz1, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(txm, 5, tym, 3, tz1, 8);
			break;
		case 2: // MIN X Z MAX Y
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, tx0, tym, tz0, txm, ty1, tzm, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(txm, 6, ty1, 8, tzm, 3);
			break;
		case 3: // MIN X MAX Y Z
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, tx0, tym, tzm, txm, ty1, tz1, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(txm, 7, ty1, 8, tz1, 8);
			break;
		case 4: // MIN Y Z MAX X
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, txm, ty0, tz0, tx1, tym, tzm, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(tx1, 8, tym, 6, tzm, 5);
			break;
		case 5: // MIN Y MAX X Z
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, txm, ty0, tzm, tx1, tym, tz1, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(tx1, 8, tym, 7, tz1, 8);
			break;
		case 6: // MIN Z MAX X Y
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, txm, tym, tz0, tx1, ty1, tzm, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = new_node(tx1, 9, ty1, 8, tzm, 7);
			break;
		case 7: // MAX X Y Z
			next = childFromLabel(node^xor);
			if (next != nullptr)
			{
				temp = next->Trace(rayOrig, rayDir, txm, tym, tzm, tx1, ty1, tz1, xor, tnear);
				if (temp != nullptr)
					return temp;
			}
			node = 8;
			break;
		}
	}
	/*for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				if (_children[i][j][k] != nullptr)
				{
					Sphere* temp = _children[i][j][k]->Trace(rayOrig, rayDir, tnear);
					if (temp != nullptr)
					{
						sphere = temp;
					}
				}*/
	return sphere;
}

int Octree::new_node(float txm, int yzExit, float tym, int xzExit, float tzm, int xyExit)
{
	if (txm < tym && txm < tzm)
		return yzExit;
	if (tym < tzm)
		return xzExit;
	else
		return xyExit;
}

Octree* Octree::childFromLabel(int label)
{
	switch (label)
	{
	case 0: // MIN X Y Z
		return _children[0][0][0];
	case 1: // MIN X Y MAX Z
		return _children[0][0][1];
	case 2: // MIN X Z MAX Y
		return _children[0][1][0];
	case 3: // MIN X MAX Y Z
		return _children[0][1][1];
	case 4: // MIN Y Z MAX X
		return _children[1][0][0];
	case 5: // MIN Y MAX X Z
		return _children[1][0][1];
	case 6: // MIN Z MAX X Y
		return _children[1][1][0];
	case 7: // MAX X Y Z
		return _children[1][1][1];
	}
}
