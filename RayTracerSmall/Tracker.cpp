#include "Tracker.h"
#include <iostream>

Tracker::Tracker()
{
    _bytes = 0;
    std::cout << "Tracker Initialised" << std::endl;
}

Tracker::~Tracker()
{
}

void Tracker::AddBytes(size_t bytes)
{
    _bytes += bytes;
    std::cout << "Allocating " << bytes << std::endl;
}

void Tracker::RemoveBytes(size_t bytes)
{
    _bytes -= bytes;
    std::cout << "Freeing " << bytes << std::endl;
}

size_t Tracker::GetByteCount()
{
    return _bytes;
}
