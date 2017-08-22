#include "ObjInGrid.h"

ObjInGrid::ObjInGrid(Point& location)
: m_location(location)
{
}

const Point & ObjInGrid::GetLocation() const
{
	return m_location;
}

void ObjInGrid::SetLocation(Point & newLocation)
{
	m_location = newLocation;
}

std::ofstream & operator<<(std::ofstream & out, const ObjInGrid & obj)
{
	out.write(reinterpret_cast<const char *>(&obj), sizeof(ObjInGrid));
	return out;
}

std::ifstream & operator>>(std::ifstream & in, ObjInGrid & obj)
{
	in.read(reinterpret_cast<char *>(&obj), sizeof(ObjInGrid));
	return in;
}
