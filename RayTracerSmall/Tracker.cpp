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
	m.lock();
	header->tracker = this;
	AddBytes(header->size);

	if (header->checkvalue != 0xDEAD)
		std::cout << "Tracker#Add: Incorrect Header checkvalue: " << header->checkvalue << " not " << 0xDEAD << std::endl;
	if (header->next || header->prev)
		std::cout << "ERROR" << std::endl;

	if (!_first)
		_first = header;

	if (_last)
	{
		if (_last->checkvalue != 0xDEAD)
			std::cout << "ERROR" << std::endl;


		bool b1 = (_last != _first && _last->prev->next == nullptr);
		//bool b2 = _last->next != nullptr;
		//bool b5 = _last->next == nullptr;
		bool b4 = _last->next;
		//bool b6 = !_last->next;

		if (b1 || b4)
		{
			//_last->next = nullptr;
			//bool b3 = _last->next != nullptr;

			std::cout << (void*)_last->next;
			std::cout << "ERROR " << (_last->next != nullptr) << " " << (_last != _first && _last->prev->next == nullptr) << std::endl;
		}
		header->prev = _last;
		_last->next = header;
	}

	_last = header;

	if (header->next)
		if (header->next->checkvalue != 0xDEAD)
			Verify(header);
	if (header->prev)
		if (header->prev->checkvalue != 0xDEAD)
			Verify(header);

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
	m.unlock();
}

void Tracker::Remove(Header* header)
{
	m.lock();
	if (header->checkvalue != 0xDEAD)
		std::cout << "Tracker#Remove: Incorrect Header checkvalue: " << header->checkvalue << " not " << 0xDEAD << std::endl;
	Footer* footer = (Footer*)(((char*)header) + sizeof(Header) + header->size);
	if (footer->checkvalue != 0xC0DE)
		std::cout << "Tracker#Remove: Incorrect Footer checkvalue: " << footer->checkvalue << " not " << 0xC0DE << std::endl;

	RemoveBytes(header->size);

	if (header->prev)
	{
		if (header->prev->checkvalue != 0xDEAD)
			Verify(header);

		if (header->next && header->next->checkvalue != 0xDEAD)
				Verify(header);

		/*if (header->next == nullptr)
		{
			header->prev->next = nullptr;
			bool b2 = header->prev->next != nullptr;
			if (b2)
				std::cout << "";
		}
		else*/
		header->prev->next = header->next;
	}
	if (header->next)
	{
		if (header->next->checkvalue != 0xDEAD)
			Verify(header);

		if (header->prev && header->prev->checkvalue != 0xDEAD)
			Verify(header);
		header->next->prev = header->prev;
	}

	if (header == _first)
		_first = header->next;

	if (header == _last)
		_last = header->prev;
	m.unlock();
}

void Tracker::Verify(Header* header)
{
	Header** pointerToHeaderPointer = &_first;
	Header* temp;
	Header* prev;

	while (*pointerToHeaderPointer)
	{
		if ((*pointerToHeaderPointer)->checkvalue != 0xDEAD)
			std::cout << "Incorrect Header checkvalue: " << temp->checkvalue << " not " << 0xDEAD << std::endl;
		temp = *pointerToHeaderPointer;
		if (temp == header)
			std::cout << "Header Found" << std::endl;

		char* mem = (char*)temp;
		Footer* footer = (Footer*)(mem + sizeof(Header) + temp->size);
		if (footer->checkvalue != 0xC0DE)
			std::cout << "Incorrect Footer checkvalue: " << footer->checkvalue << " not " << 0xC0DE << std::endl;
		prev = temp;
		pointerToHeaderPointer = &(temp->next);

		if ((*pointerToHeaderPointer) && !(*pointerToHeaderPointer)->prev)
			std::cout << "ERROR Link Broke Forwards" << std::endl;
	}

	if (temp != _last)
	{
		std::cout << "Last pointer found is not the Last" << std::endl;


		Header** hPP = &_last;
		Header* h;
		Header* p;

		while (*hPP)
		{
			if ((*hPP)->checkvalue != 0xDEAD)
				std::cout << "Incorrect Header checkvalue: " << h->checkvalue << " not " << 0xDEAD << std::endl;
			h = *hPP;
			if (h == header)
				std::cout << "Header Found" << std::endl;

			char* mem = (char*)h;
			Footer* footer = (Footer*)(mem + sizeof(Header) + h->size);
			if (footer->checkvalue != 0xC0DE)
				std::cout << "Incorrect Footer checkvalue: " << footer->checkvalue << " not " << 0xC0DE << std::endl;
			p = h;
			hPP = &(h->prev);

			if (!(*hPP)->next)
				std::cout << "ERROR Link Broke Backwards" << std::endl;
		}

		if (*hPP != _first)
			std::cout << "First pointer found is not the First" << std::endl;
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
