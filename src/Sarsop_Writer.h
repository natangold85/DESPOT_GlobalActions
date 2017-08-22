#pragma once

#include <vector>
#include <map>
#include <Windows.h>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

class Sarsop_Writer
{
	// operator comparison for using pomdp as key in map
	friend bool operator<(const Sarsop_Writer& p1, const Sarsop_Writer& p2);
	// operator << to print the pomdp
	friend std::ostream& operator<<(std::ostream& o, const Sarsop_Writer& map);
	// operator << to stream in and out the pomdp
	friend std::ofstream& operator<<(std::ofstream& out, const Sarsop_Writer& map);
	friend std::ifstream& operator>>(std::ifstream& in, Sarsop_Writer& map);

public:
	explicit Sarsop_Writer() = default;
	~Sarsop_Writer() = default;
	Sarsop_Writer(const Sarsop_Writer &) = default;
	Sarsop_Writer(Sarsop_Writer&&) = default;
	Sarsop_Writer& operator=(const Sarsop_Writer&) = default;

	// return number of moving objects in grid
	size_t CountMovableObj() const;
	size_t CountEnemies() const;
	size_t CountNInv() const;
	size_t CountShelters() const;
	size_t GetGridSize() const;

private:
	size_t m_gridSize;
	size_t m_targetIdx;
	Self_Obj m_self;
	double m_discount;

	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_NInvVector;
	std::vector<ObjInGrid> m_shelter;
};

