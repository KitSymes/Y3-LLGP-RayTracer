#include "Tracker.h"
#include "Structures.h"
#include <iostream>

Tracker::Tracker()
{
	_bytes = 0;
	if (DEBUG)
		std::cout << "Tracker Initialised" << std::endl;
	_first = nullptr;
	_last = nullptr;
}

Tracker::~Tracker()
{
}

void Tracker::Add(Header* header)
{
	header->tracker = this;
	AddBytes(header->size);

	if (_first == nullptr)
		_first = header;

	if (_last != nullptr)
	{
		_last->next = header;
		header->prev = _last;
	}

	_last = header;
	/*// Pointing to a header pointer, so it's pointing to _first or ->next
	Header** pointerToHeaderPointer = &_first;
	Header* pointerToPreviousHeader = nullptr;

	while (*pointerToHeaderPointer != nullptr)
	{
		pointerToPreviousHeader = *pointerToHeaderPointer;
		pointerToHeaderPointer = &pointerToPreviousHeader->next;
	}
	(*pointerToHeaderPointer) = header;
	if (pointerToPreviousHeader != nullptr)
		header->prev = pointerToPreviousHeader;*/
}

void Tracker::Remove(Header* header)
{
	if (header->checkvalue != 0xDEAD)
		std::cout << "Remove Incorrect Header checkvalue: " << header->checkvalue << " not " << 0xDEAD << std::endl;
	Footer* footer = (Footer*)(((char*)header)+sizeof(Header) + header->size);
	if (footer->checkvalue != 0xC0DE)
		std::cout << "Incorrect Footer checkvalue: " << footer->checkvalue << " not " << 0xC0DE << std::endl;
	RemoveBytes(header->size);
	if (header->prev != nullptr)
		header->prev->next = header->next;
	if (header->next != nullptr)
		header->next->prev = header->prev;
	if (header == _first)
		_first = header->next;
}

void Tracker::Verify()
{
	Header** pointerToHeaderPointer = &_first;
	Header* temp;

	while (*pointerToHeaderPointer != nullptr)
	{
		temp = *pointerToHeaderPointer;
		if (temp->checkvalue != 0xDEAD)
			std::cout << "Incorrect Header checkvalue: " << temp->checkvalue << " not " << 0xDEAD << std::endl;
		char* mem = (char*)temp;
		Footer* footer = (Footer*)(mem + sizeof(Header) + temp->size);
		if (footer->checkvalue != 0xC0DE)
			std::cout << "Incorrect Footer checkvalue: " << footer->checkvalue << " not " << 0xC0DE << std::endl;
		pointerToHeaderPointer = &temp->next;
	}

}

size_t Tracker::GetByteCount()
{
	return _bytes;
}

void Tracker::AddBytes(size_t bytes)
{
	_bytes += bytes;
	if (DEBUG)
		std::cout << "Allocating " << bytes << std::endl;
}

void Tracker::RemoveBytes(size_t bytes)
{
	_bytes -= bytes;
	if (DEBUG)
		std::cout << "Freeing " << bytes << std::endl;
}
