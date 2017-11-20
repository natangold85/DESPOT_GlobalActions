#include "Coordinate.h"

Coordinate::Coordinate(int x, int y)
: m_x(x)
, m_y(y)
{
}

Coordinate::Coordinate(std::pair<int, int> coord)
: m_x(coord.first)
, m_y(coord.second)
{}

Coordinate &Coordinate::operator=(std::pair<int, int> coord)
{
	m_x = coord.first;
	m_y = coord.second;

	return *this;
}
/// return min(a,b)
inline int Min(int a, int b)
{
	return a * (a <= b) + b * (b < a);
}

/// return min(a,b)
inline int Max(int a, int b)
{
	return a * (a >= b) + b * (b > a);
}

/// return the minimum pair min(first), min(second)
Coordinate Min(Coordinate & a, Coordinate & b)
{
	Coordinate ret;
	ret.m_x = Min(a.m_x, b.m_x);
	ret.m_y = Min(a.m_y, b.m_y);
	return ret;
}

/// return the maximum pair max(first), max(second)
Coordinate Max(Coordinate & a, Coordinate & b)
{
	Coordinate ret;
	ret.m_x = Max(a.m_x, b.m_x);
	ret.m_y = Max(a.m_y, b.m_y);
	return ret;
}

/// return the minimum pair min(first), min(second)
Coordinate Min(std::vector<Coordinate> & location, int idx)
{
	Coordinate min = location[0];
	for (int i = 1; i < idx; ++i)
	{
		min = Min(min, location[i]);
	}

	return min;
}

/// return the maximum pair max(first), max(second)
Coordinate Max(std::vector<Coordinate> & location, int idx)
{
	Coordinate max = location[0];
	for (int i = 1; i < idx; ++i)
	{
		max = Max(max, location[i]);
	}

	return max;
}

void Coordinate::Zero()
{
	m_x = 0;
	m_y = 0;
}

;

Coordinate& Coordinate::operator-=(const Coordinate toDecrease)
{
	m_x -= toDecrease.m_x;
	m_y -= toDecrease.m_y;
	return *this;
}

Coordinate& Coordinate::operator+=(const Coordinate toIncrease)
{
	m_x += toIncrease.m_x;
	m_y += toIncrease.m_y;
	return *this;
}

Coordinate& Coordinate::operator/=(double toDivide)
{
	m_x /= toDivide;
	m_y /= toDivide;
	return *this;
}

Coordinate & Coordinate::operator*=(double toMultiply)
{
	m_x *= toMultiply;
	m_y *= toMultiply;
	return *this;
}


Coordinate Coordinate::operator-(Coordinate & toDecrease)
{
	Coordinate ret(m_x, m_y);
	return ret -= toDecrease;
}

Coordinate Coordinate::operator+(Coordinate & toIncrease)
{
	Coordinate ret(m_x, m_y);
	return ret -= toIncrease;
}

Coordinate Coordinate::operator/(double toDivide)
{
	Coordinate ret(m_x, m_y);
	return ret /= toDivide;
}

bool Coordinate::operator==(const Coordinate & a) const
{
	return (m_x == a.m_x) & (m_y == a.m_y);
}

bool Coordinate::operator!=(const Coordinate & a) const
{
	return !(*this == a);
}

int Coordinate::Distance(const Coordinate & num) const
{
	int xDiff = m_x - num.m_x;
	int yDiff = m_y - num.m_y;
	return xDiff * xDiff + yDiff * yDiff;
}

double Coordinate::RealDistance(const Coordinate & a) const
{
	int xDiff = m_x - a.m_x;
	int yDiff = m_y - a.m_y;

	return sqrt(xDiff * xDiff + yDiff * yDiff);
}