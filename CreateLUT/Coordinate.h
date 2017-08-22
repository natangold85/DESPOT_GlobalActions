#pragma once

#include <vector>

class Coordinate
{
public:
	Coordinate() = default;
	Coordinate(int x, int y);
	~Coordinate() = default;

	friend Coordinate Min(Coordinate & a, Coordinate & b);
	friend Coordinate Min(std::vector<Coordinate> & location, int idx);
	friend Coordinate Max(Coordinate & a, Coordinate & b);
	friend Coordinate Max(std::vector<Coordinate> & location, int idx);

	void Zero();
	Coordinate& operator-=(const Coordinate toDecrease);
	Coordinate& operator+=(const Coordinate toIncrease);
	Coordinate& operator/=(double toDivide);
	Coordinate& operator*=(double toMultiply);

	bool operator==(const Coordinate & a) const;
	bool operator!=(const Coordinate & a) const;

	Coordinate operator-(Coordinate & toDecrease);
	Coordinate operator+(Coordinate & toIncrease);
	Coordinate operator/(double toDivide);

	int Distance(const Coordinate &) const;
	
	int &X() { return m_x; };
	int &Y() { return m_y; };

	int X() const { return m_x; };
	int Y() const { return m_y; };

private:
	int m_x;
	int m_y;
};

