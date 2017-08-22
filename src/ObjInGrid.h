#pragma once
#include <fstream>      // std::ofstream

#include "Point.h"

/// a base class for objects in grid for pomdp project. can be used as shelter
class ObjInGrid
{
public:
	explicit ObjInGrid() = default;
	explicit ObjInGrid(Point& location);
	virtual ~ObjInGrid() = default;
	ObjInGrid(const ObjInGrid&) = default;
	ObjInGrid& operator=(const ObjInGrid&) = default;

	/// Get location of object
	const Point &GetLocation() const;
	/// Set location of object
	void SetLocation(Point &newLocation);
	
	/// write object to file
	friend std::ofstream& operator<<(std::ofstream& out, const ObjInGrid& obj);
	/// read object from file
	friend std::ifstream& operator>>(std::ifstream& in, ObjInGrid& obj);

private:
	Point m_location;
};



