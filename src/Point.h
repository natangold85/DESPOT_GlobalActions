#pragma once
#include <fstream>      // std::ofstream

/// a point on grid. when std is distribution of lcations around the point
class Point
{
public:
	explicit Point();
	explicit Point(int x, int y, double m_std = 0.0);
	~Point() = default;

	int GetX() const;
	int GetY() const;
	/// translate (x,y) to idx given a grid size
	int GetIdx(int gridSize) const;
	void SetIdx(int idx, int gridSize);
	double GetStd() const;

	/// write to file
	friend std::ofstream& operator<<(std::ofstream& out, const Point& obj);
	/// read from file
	friend std::ifstream& operator>>(std::ifstream& in, Point& obj);
private:
	int m_x;
	int m_y;
	double m_std;	//standard deviation for the point
};

