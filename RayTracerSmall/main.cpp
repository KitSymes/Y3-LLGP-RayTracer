// [header]
// A very basic raytracer example.
// [/header]
// [compile]
// c++ -o raytracer -O3 -Wall raytracer.cpp
// [/compile]
// [ignore]
// Copyright (C) 2012  www.scratchapixel.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// [/ignore]
#include <stdlib.h>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>
// Windows only
#include <algorithm>
#include <sstream>
#include <string.h>
// Additional
#include <json.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include "TrackerManager.h"
#include "Vec3.h"
#include "Sphere.h"
#include "Structures.h"
#include "Octree.h"

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif

//[comment]
// This variable controls the maximum recursion depth
//[/comment]
#define MAX_RAY_DEPTH 5
// This controls how many trace threads are generated, as there is one for every x lines
#define TRACE_THREAD_PER_LINES 50
#define SUBDIVIDE_COUNT 3
#define DEBUG false
#define OCTREE true
#define COUNT 10

float mix(const float& a, const float& b, const float& mix)
{
	return b * mix + a * (1 - mix);
}

//[comment]
// This is the main trace function. It takes a ray as argument (defined by its origin
// and direction). We test if this ray intersects any of the geometry in the scene.
// If the ray intersects an object, we compute the intersection point, the normal
// at the intersection point, and shade this point using this information.
// Shading depends on the surface property (is it transparent, reflective, diffuse).
// The function returns a color for the ray. If the ray intersects an object that
// is the color of the object at the intersection point, otherwise it returns
// the background color.
//[/comment]
Vec3f trace(
	const Vec3f& rayOrig,
	const Vec3f& rayDir,
	const std::vector<Sphere*>& spheres,
	const int& depth,
	Octree* oct)
{
	//if (raydir.length() != 1) std::cerr << "Error " << raydir << std::endl;
	float tnear = INFINITY;
	const Sphere* sphere = NULL;
	// find intersection of this ray with the sphere in the scene

#if OCTREE
	unsigned char xor = 0;
	Vec3f dir = rayDir;
	Vec3f orig = rayOrig;

	if (rayDir.x < 0.0)
	{
		orig.x = oct->GetCentre().x - rayOrig.x;
		dir.x = -rayDir.x;
		xor |= 4;
	}
	if (rayDir.y < 0.0)
	{
		orig.y = oct->GetCentre().y -rayOrig.y;
		dir.y = -rayDir.y;
		xor |= 2;
	}
	if (rayDir.z < 0.0)
	{
		orig.z = oct->GetCentre().z -rayOrig.z;
		dir.z = -rayDir.z;
		xor |= 1;
	}

	float tx0 = (oct->xmin - orig.x) / dir.x;
	float tx1 = (oct->xmax - orig.x) / dir.x;
	float ty0 = (oct->ymin - orig.y) / dir.y;
	float ty1 = (oct->ymax - orig.y) / dir.y;
	float tz0 = (oct->zmin - orig.z) / dir.z;
	float tz1 = (oct->zmax - orig.z) / dir.z;

	if (std::max(std::max(tx0, ty0), tz0) < std::min(std::min(tx1, ty1), tz1))
		sphere = oct->Trace(rayOrig, rayDir, tx0, ty0, tz0, tx1, ty1, tz1, xor, tnear);
#else
	for (unsigned i = 0; i < spheres.size(); ++i) {
		float t0 = INFINITY, t1 = INFINITY;
		if (spheres[i]->intersect(rayOrig, rayDir, t0, t1)) {
			if (t0 < 0) t0 = t1;
			if (t0 < tnear) {
				tnear = t0;
				sphere = spheres[i];
			}
		}
	}
#endif
	// if there's no intersection return black or background color
	if (!sphere) return Vec3f(2);
	Vec3f surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray
	Vec3f phit = rayOrig + rayDir * tnear; // point of intersection
	Vec3f nhit = phit - sphere->center; // normal at the intersection point
	nhit.normalize(); // normalize normal direction
	// If the normal and the view direction are not opposite to each other
	// reverse the normal direction. That also means we are inside the sphere so set
	// the inside bool to true. Finally reverse the sign of IdotN which we want
	// positive.
	float bias = 1e-4; // add some bias to the point from which we will be tracing
	bool inside = false;
	if (rayDir.dot(nhit) > 0) nhit = -nhit, inside = true;
	if ((sphere->transparency > 0 || sphere->reflection > 0) && depth < MAX_RAY_DEPTH) {
		float facingratio = -rayDir.dot(nhit);
		// change the mix value to tweak the effect
		float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);
		// compute reflection direction (not need to normalize because all vectors
		// are already normalized)
		Vec3f refldir = rayDir - nhit * 2 * rayDir.dot(nhit);
		refldir.normalize();
		Vec3f reflection = trace(phit + nhit * bias, refldir, spheres, depth + 1, oct);
		Vec3f refraction = 0;
		// if the sphere is also transparent compute refraction ray (transmission)
		if (sphere->transparency) {
			float ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
			float cosi = -nhit.dot(rayDir);
			float k = 1 - eta * eta * (1 - cosi * cosi);
			Vec3f refrdir = rayDir * eta + nhit * (eta * cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(phit - nhit * bias, refrdir, spheres, depth + 1, oct);
		}
		// the result is a mix of reflection and refraction (if the sphere is transparent)
		surfaceColor = (
			reflection * fresneleffect +
			refraction * (1 - fresneleffect) * sphere->transparency) * sphere->surfaceColor;
	}
	else {
		// it's a diffuse object, no need to raytrace any further
		for (unsigned i = 0; i < spheres.size(); ++i) {
			if (spheres[i]->emissionColor.x > 0) {
				// this is a light
				Vec3f transmission = 1;
				Vec3f lightDirection = spheres[i]->center - phit;
				lightDirection.normalize();
				for (unsigned j = 0; j < spheres.size(); ++j) {
					if (i != j) {
						float t0, t1;
						if (spheres[j]->intersect(phit + nhit * bias, lightDirection, t0, t1)) {
							transmission = 0;
							break;
						}
					}
				}
				surfaceColor += sphere->surfaceColor * transmission *
					std::max(float(0), nhit.dot(lightDirection)) * spheres[i]->emissionColor;
			}
		}
	}

	return surfaceColor + sphere->emissionColor;
}

void traceThreadedOld(const std::vector<Sphere*>& spheres, Vec3f* image, unsigned int start, unsigned width, unsigned height, float invWidth, float invHeight, float aspectRatio, float angle, Octree* oct)
{
	for (unsigned y = start; y < std::min(start + TRACE_THREAD_PER_LINES, height); ++y) {
		for (unsigned x = 0; x < width; ++x) {
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectRatio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
			Vec3f raydir(xx, yy, -1);
			raydir.normalize();
			Vec3f pixel = trace(Vec3f(0), raydir, spheres, 0, oct);
			//mutex.lock();
			image[int(x + y * width)] = pixel;
			//mutex.unlock();
		}
	}
}

void traceThreaded(const std::vector<Sphere*>& spheres, Vec3f* image,
	unsigned int startX, unsigned int startY, unsigned int sizeX, unsigned int sizeY,
	unsigned width, unsigned height, float invWidth, float invHeight, float aspectRatio, float angle,
	Octree* oct)
{
	for (unsigned y = startY; y < height && y < startY + sizeY; ++y) {
		for (unsigned x = startX; x < width && x < startX + sizeX; ++x) {
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectRatio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
			Vec3f raydir(xx, yy, -1);
			raydir.normalize();
			Vec3f pixel = trace(Vec3f(0), raydir, spheres, 0, oct);
			image[int(x + y * width)] = pixel;
		}
	}
}

//[comment]
// Main rendering function. We compute a camera ray for each pixel of the image
// trace it and return a color. If the ray hits a sphere, we return the color of the
// sphere at the intersection point, else we return the background color.
//[/comment]
void render(const std::vector<Sphere*>& spheres, int iteration)
{
	std::mutex mutex;

	Octree* oct = new Octree(Vec3f());
	oct->Create(spheres, 50.0f);

	// Recommended Testing Resolution
	unsigned width = 640, height = 480;

	// Recommended Production Resolution
	//unsigned width = 1920, height = 1080;

	Vec3f* image = new Vec3f[width * height], * pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 30, aspectratio = width / float(height);
	float angle = tan(M_PI * 0.5 * fov / 180.);

	std::vector<std::thread> traceThreads;

	int threads = pow(4, SUBDIVIDE_COUNT);
	int split = pow(2, SUBDIVIDE_COUNT);
	//int threads = pow(SUBDIVIDE_COUNT + 1, 2);
	//int split = SUBDIVIDE_COUNT + 1;

#if DEBUG
	traceThreads.push_back(std::thread(traceThreaded, std::cref(spheres), std::ref(image),
		457, 50, 5, 5,
		width, height, invWidth, invHeight, aspectratio, angle,
		oct));
#else
	for (unsigned int i = 0; i < threads; i++)
	{
		float sizeX = width / split;
		float sizeY = height / split;
		float startX = (i / split) * sizeX;
		float startY = (i % split) * sizeY;
		traceThreads.push_back(std::thread(traceThreaded, std::cref(spheres), std::ref(image),
			startX, startY, sizeX, sizeY,
			width, height, invWidth, invHeight, aspectratio, angle,
			oct));
	}
#endif

	for (std::thread& t : traceThreads)
	{
		if (t.joinable())
			t.join();
	}

	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::stringstream ss;
	ss << "./spheres" << iteration << ".ppm";
	std::string tempString = ss.str();
	char* filename = (char*)tempString.c_str();
	std::ofstream ofs(filename, std::ios::out | std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";

	// 94460 average micro seconds
	for (unsigned i = 0; i < width * height; ++i) {
		ofs << (unsigned char)(std::min(float(1), image[i].x) * 255) <<
			(unsigned char)(std::min(float(1), image[i].y) * 255) <<
			(unsigned char)(std::min(float(1), image[i].z) * 255);
	}

	ofs.close();

	delete[] image;
	delete oct;
}

void LoadScene(std::vector<Sphere*>& spheres)
{
	std::ifstream f("scene.json");
	nlohmann::json data = nlohmann::json::parse(f);

	for (nlohmann::json::iterator it = data["spheres"].begin(); it != data["spheres"].end(); ++it) {
		auto sphere = it.value();
		Vec3f centre = Vec3f(sphere["centre"][0], sphere["centre"][1], sphere["centre"][2]);
		Vec3f surfaceColor = Vec3f(sphere["surfaceColor"][0], sphere["surfaceColor"][1], sphere["surfaceColor"][2]);
		Vec3f emissionColor = Vec3f(sphere["emissionColor"][0], sphere["emissionColor"][1], sphere["emissionColor"][2]);
		spheres.push_back(new Sphere(centre,
			sphere["radius"],
			surfaceColor,
			sphere["reflection"],
			sphere["transparency"],
			emissionColor));
	}
}

void BasicRender()
{
	std::vector<Sphere*> spheres;
	// Vector structure for Sphere (position, radius, surface color, reflectivity, transparency, emission color)

	LoadScene(spheres);
	//TrackerManager::GetInstance().GetDefaultTracker()->Verify();

	// This creates a file, titled 1.ppm in the current working directory
	render(spheres, 1);

	for (Sphere* sphere : spheres)
		delete sphere;
}

void SimpleShrinking()
{
	std::vector<Sphere*> garbage;
	std::vector<std::thread> renderThreads;
	// Vector structure for Sphere (position, radius, surface color, reflectivity, transparency, emission color)

	for (int i = 0; i < 4; i++)
	{
		std::vector<Sphere*> spheres;
		LoadScene(spheres);

		if (i == 0)
		{
			spheres[1]->radius = 4;
			spheres[1]->radius2 = 16;
		}
		else if (i == 1)
		{
			spheres[1]->radius = 3;
			spheres[1]->radius2 = 9;
		}
		else if (i == 2)
		{
			spheres[1]->radius = 2;
			spheres[1]->radius2 = 4;
		}
		else if (i == 3)
		{
			spheres[1]->radius = 1;
			spheres[1]->radius2 = 1;
		}

		renderThreads.push_back(std::thread(render, spheres, i));
		garbage.insert(std::end(garbage), std::begin(spheres), std::end(spheres));
	}

	for (std::thread& thread : renderThreads)
		if (thread.joinable())
			thread.join();

	for (Sphere* sphere : garbage)
		delete sphere;
}

void SmoothScaling()
{
	std::vector<Sphere*> garbage;
	std::vector<std::thread> renderThreads;

	for (float r = 0; r <= 100; r++)
	{
		std::vector<Sphere*> spheres;
		LoadScene(spheres);

		spheres[1]->radius = r / 100;
		spheres[1]->radius2 = (r / 100) * (r / 100);
		renderThreads.push_back(std::thread(render, spheres, r));
		garbage.insert(std::end(garbage), std::begin(spheres), std::end(spheres));
	}

	for (std::thread& thread : renderThreads)
		if (thread.joinable())
			thread.join();

	for (Sphere* sphere : garbage)
		delete sphere;
}

//[comment]
// In the main function, we will create the scene which is composed of 5 spheres
// and 1 light (which is also a sphere). Then, once the scene description is complete
// we render that scene, by calling the render() function.
//[/comment]
int main(int argc, char** argv)
{
	// This sample only allows one choice per program execution. Feel free to improve upon this
	srand(13);
	//BasicRender();
	//SimpleShrinking();
	//SmoothScaling();


	using std::chrono::high_resolution_clock;
	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::milliseconds;

	int count = COUNT;
	int total = 0;
	for (int i = 0; i < count; i++)
	{
		auto t1 = high_resolution_clock::now();
		BasicRender();
		auto t2 = high_resolution_clock::now();

		auto t_int = duration_cast<milliseconds>(t2 - t1);

		std::cout << t_int.count() << " ms seconds\n";
		total += t_int.count();
	}

	std::cout << (total / count) << " average ms seconds\n";


	return 0;
}

void* operator new(size_t size)
{
	size_t bytes = size + sizeof(Header) + sizeof(Footer);
	//std::cout << "new " << size << " reqeusted, allocating " << bytes << std::endl;
	char* pMem = (char*)malloc(bytes);

	Header* header = (Header*)pMem;
	header->size = size;
	header->next = nullptr;
	header->prev = nullptr;
	header->checkvalue = 0xDEAD;
	TrackerManager::GetInstance().GetDefaultTracker()->Add(header);

	Footer* footer = (Footer*)(pMem + sizeof(Header) + size);
	footer->checkvalue = 0xC0DE;

	void* pStartMemBlock = pMem + sizeof(Header);
	return pStartMemBlock;
}

void* operator new(size_t size, Tracker* tracker)
{
	size_t bytes = size + sizeof(Header) + sizeof(Footer);
	//std::cout << "new " << size << " reqeusted, allocating " << bytes << std::endl;
	char* pMem = (char*)malloc(bytes);

	Header* header = (Header*)pMem;
	header->size = size;
	header->next = nullptr;
	header->prev = nullptr;
	header->checkvalue = 0xDEAD;
	tracker->Add(header);

	Footer* footer = (Footer*)(pMem + sizeof(Header) + size);
	footer->checkvalue = 0xC0DE;

	void* pStartMemBlock = pMem + sizeof(Header);
	return pStartMemBlock;
}

void operator delete(void* pMem)
{
	Header* header = (Header*)((char*)pMem - sizeof(Header));
	Footer* footer = (Footer*)((char*)pMem + header->size);

	//std::cout << "freeing " << header->size << " (" << (header->size + sizeof(Header) + sizeof(Footer)) << ")" << std::endl;
	header->tracker->Remove(header);
	free(header);
}
