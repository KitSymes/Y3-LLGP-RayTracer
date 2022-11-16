#include "TrackerManager.h"
#include <iostream>

TrackerManager& TrackerManager::GetInstance()
{
	static TrackerManager instance;
	return instance;
}

Tracker* TrackerManager::GetDefaultTracker()
{
	std::cout << "Default Tracker: ";
	return &_defaultTracker;
}

Tracker* TrackerManager::GetSphereTracker()
{
	std::cout << "Sphere Tracker: ";
	return &_sphereTracker;
}

TrackerManager::TrackerManager()
{
	std::cout << "Tracker Manager Initialised" << std::endl;
}
