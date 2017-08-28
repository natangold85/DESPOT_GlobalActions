#include "ObjInGrid.h"

ObjInGrid::ObjInGrid(Coordinate& location)
: m_location(location)
{
}

const Coordinate & ObjInGrid::GetLocation() const
{
	return m_location;
}

void ObjInGrid::SetLocation(Coordinate & newLocation)
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
