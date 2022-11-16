#pragma once
#include "Tracker.h"
class TrackerManager
{
public:
	static TrackerManager& GetInstance();

	Tracker* GetDefaultTracker();
	Tracker* GetSphereTracker();
private:
	TrackerManager();

	Tracker _defaultTracker;
	Tracker _sphereTracker;
};

