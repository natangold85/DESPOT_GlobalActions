#include "Point.h"

Point::Point()
	: m_x(0)
	, m_y(0)
	, m_std(0.0)
{
}

Point::Point(int x, int y, double std)
: m_x(x)
, m_y(y)
, m_std(std)
{
}

int Point::GetX() const
{
	return m_x;
}

int Point::GetY() const
{
	return m_y;
}

int Point::GetIdx(int gridSize) const
{
	return m_y * gridSize + m_x;
}

void Point::SetIdx(int idx, int gridSize)
{
	m_x = idx % gridSize;
	m_y = idx / gridSize;
}

double Point::GetStd() const
{
	return m_std;
}

std::ofstream & operator<<(std::ofstream & out, const Point & obj)
{
	out << obj.m_x;
	out << obj.m_y;
	out << obj.m_std;

	return out;
}

std::ifstream & operator>>(std::ifstream & in, Point & obj)
{
	in >> obj.m_x;
	in >> obj.m_y;
	in >> obj.m_std;

	return in;
}
