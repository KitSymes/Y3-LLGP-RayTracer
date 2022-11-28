#pragma once
#ifndef _SPHERE_H
#define _SPHERE_H

#include "Vec3.h"

typedef Vec3<float> Vec3f;

class Sphere
{
public:
	Vec3f center;                           /// position of the sphere
	float radius, radius2;                  /// sphere radius and radius^2
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity
	Sphere(
		const Vec3f& c,
		const float& r,
		const Vec3f& sc,
		const float& refl = 0,
		const float& transp = 0,
		const Vec3f& ec = 0) :
		center(c), radius(r), radius2(r* r), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl)
	{ /* empty */
	}

	//[comment]
	// Compute a ray-sphere intersection using the geometric solution
	//[/comment]
	bool intersect(const Vec3f& rayorig, const Vec3f& raydir, float& t0, float& t1) const
	{
		Vec3f l = center - rayorig;
		float tca = l.dot(raydir);
		if (tca < 0) return false;
		float d2 = l.dot(l) - tca * tca;
		if (d2 > radius2) return false;
		float thc = sqrt(radius2 - d2);
		t0 = tca - thc;
		t1 = tca + thc;

		return true;
	}

	// Analytic solution https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
	bool intersectAnalytic(const Vec3f& rayorig, const Vec3f& raydir, float& t0, float& t1) const
	{
		Vec3f L = rayorig - center;
		float a = raydir.dot(raydir);
		float b = 2 * raydir.dot(L);
		float c = L.dot(L) - radius2;

		if (!solveQuadratic(a, b, c, t0, t1)) return false;

		if (t0 < 0 && t1 < 0) return false;

		return true;
	}

	bool solveQuadratic(float& a, float& b, float& c, float& x0, float& x1) const
	{
		float discr = b * b - 4 * a * c;
		if (discr < 0) return false;
		else if (discr == 0) x0 = x1 = -0.5 * b / a;
		else {
			float q = (b > 0) ?
				-0.5 * (b + sqrt(discr)) :
				-0.5 * (b - sqrt(discr));
			x0 = q / a;
			x1 = c / q;
		}
		if (x0 > x1) std::swap(x0, x1);

		return true;
	}

	bool IntersectsLine(const Vec3f& rayorig, const Vec3f& raydir, float& distanceSq)
	{
		// Closest point on the line to position
		Vec3f AB = raydir;
		Vec3f AP = center - rayorig;
		float lengthSqAB = AB.x * AB.x + AB.y * AB.y;
		float t = (AP.x * AB.x + AP.y * AB.y) / lengthSqAB;
		if (t < 0)
			t = 0;
		if (t > 1)
			t = 1;

		Vec3f closest = rayorig + AB * t;

		if ((closest - center).length2() > radius2)
			return false;

		distanceSq = closest.length2();

		return true;
	}

	bool IntersectsLine(Vec3f start, Vec3f end)
	{
		// Closest point on the line to position (includes past start and end point), but that shouldn't matter
		Vec3f AB = end - start;
		Vec3f AP = center - start;
		float lengthSqrAB = AB.x * AB.x + AB.y * AB.y;
		float t = (AP.x * AB.x + AP.y * AB.y) / lengthSqrAB;
		if (t < 0)
			t = 0;
		if (t > 1)
			t = 1;
		Vec3f closest = start + AB * t;

		Vec3f temp = closest - center;
		float mag = (temp.x * temp.x) + (temp.y * temp.y) + (temp.z * temp.z);

		return mag <= radius2;
	}
};
#endif
