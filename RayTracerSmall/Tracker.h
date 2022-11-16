#pragma once
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

