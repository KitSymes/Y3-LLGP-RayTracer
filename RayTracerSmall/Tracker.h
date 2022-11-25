#pragma once
#ifndef _TRACKER_H
#define _TRACKER_H

#define DEBUG false
class Tracker
{
public:
	Tracker();
	~Tracker();

	void AddBytes(size_t bytes);
	void RemoveBytes(size_t bytes);

	size_t GetByteCount();
private:
	size_t _bytes;
};
#endif
