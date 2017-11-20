#include <cmath>		//sqrt
#include <algorithm>    // std::sort

#include "Attacks.h"

inline int Abs(int num)
{
	return num * (num >= 0) - num * (num < 0);
}

inline int Sign(int num)
{
	return 1 * (num >= 0) - 1 * (num < 0);
}

inline Coordinate Abs(Coordinate num)
{
	num.X() = Abs(num.X());
	num.Y() = Abs(num.Y());
	return num;
}

inline Coordinate Sign(Coordinate num)
{
	num.X() = Sign(num.X());
	num.Y() = Sign(num.Y());
	return num;
}

inline std::pair<double, double> & operator+=(std::pair<double, double> & a, std::pair<double, double> & b)
{
	a.first += b.first;
	a.second += b.second;
	return a;
}

inline std::pair<double, double> operator+(std::pair<double, double> & a, std::pair<double, double> & b)
{
	std::pair<double, double> ret(a);
	ret.first += b.first;
	ret.second += b.second;

	return ret;
}

inline double Distance(std::pair<double, double> & a, std::pair<double, double> & b)
{
	std::pair<double, double> diff = std::make_pair(a.first - b.first, a.second - b.second);
	return sqrt(diff.first * diff.first + diff.second * diff.second);
}


inline void Swap(int &a, int &b)
{
	int c = a;
	a = b;
	b = c;
}

DirectAttack::DirectAttack(double range, double pHit)
	: m_range(range)
	, m_pHit(pHit)
{
}

void DirectAttack::AttackOnline(int attackerLoc, int targetLoc, intVec & state, intVec & shelterLoc, int gridSize, double random) const
{
	Coordinate attacker(attackerLoc % gridSize, attackerLoc / gridSize);
	Coordinate target(targetLoc % gridSize, targetLoc / gridSize);
	double dist = target.RealDistance(attacker);

	if (dist <= m_range & dist > 0)
	{
		// calculate attack result
		shootOutcomes result;
		CalcAttackResult(attacker, target, state, shelterLoc, gridSize, result);
		// run on result and return true if self object killed
		for (auto v : result)
		{
			random -= v.second;
			if (random <= 0.0)
			{
				state = v.first;
				return;
			}
		}
	}
	return;
}

void DirectAttack::AttackOffline(int attackerLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, shootOutcomes & result) const
{
	Coordinate attacker(attackerLoc % gridSize, attackerLoc / gridSize);
	Coordinate target(targetLoc % gridSize, targetLoc / gridSize);
	// check outcomes for enemy as attacker, don't treat cases when enemy kill non involved
	CalcAttackResult(attacker, target, state, shelters, gridSize, result);
}

//void DirectAttack::CalcSelfAttackOffline(intVec & state, intVec & shelters, int gridSize, shootOutcomes & result) const
//{
//	// TODO: make self attack also directed to observation
//	// the attack is allways directed to enemy (location 1)
//	intVec fightingObj{ state[0], state[1] };
//	CalcAttackResult(state, shelters, fightingObj, gridSize, result);
//
//	return;
//}

bool DirectAttack::InRange(int location, int otherObjLocation, int gridSize) const
{
	Coordinate self(location % gridSize, location / gridSize);
	Coordinate object(otherObjLocation % gridSize, otherObjLocation / gridSize);

	return self.RealDistance(object) <= m_range;
}

std::string DirectAttack::String() const
{
	std::string ret = "Direct Attack: range = ";
	ret += std::to_string(m_range) + " pHit = " + std::to_string(m_pHit);
	return ret;
}

void DirectAttack::CalcAttackResult(Coordinate & attacker, Coordinate & target, intVec state, intVec shelters, int gridSize, shootOutcomes & result) const
{
	std::pair<double, double> change;
	CalcChanges(attacker, target, change);

	std::pair<double, double> selfLocation(attacker.X(), attacker.Y());
	std::pair<double, double> currLocation(attacker.X(), attacker.Y());
	Coordinate prevLocation(attacker);
	Coordinate location;

	while (prevLocation != target)
	{
		prevLocation = currLocation;
		currLocation += change;
		location = currLocation;


		int idxLocation = currLocation.first + currLocation.second * gridSize;

		for (auto v : shelters)
		{
			if (idxLocation == v)
			{
				double pLeft = CalcDiversion(state, shelters, location, gridSize, prevLocation, result);
				result.emplace_back(std::make_pair(state, m_pHit + pLeft));
				return;
			}
		}

		for (auto v = state.begin(); v != state.end(); ++v)
		{
			if (idxLocation == *v)
			{
				double pLeft = CalcDiversion(state, shelters, location, gridSize, prevLocation, result);
				result.emplace_back(std::make_pair(state, pLeft));
				*v = gridSize * gridSize;
				result.emplace_back(std::make_pair(state, m_pHit));
				return;
			}
		}

		if (Distance(selfLocation, currLocation + change) > m_range)
		{
			double pLeft = CalcDiversion(state, shelters, location, gridSize, prevLocation, result);
			result.emplace_back(std::make_pair(state, m_pHit + pLeft));
			return;
		}

	}
}

void DirectAttack::CalcChanges(Coordinate obj1, Coordinate obj2, std::pair<double, double> & change)
{
	if (obj1.X() == obj2.X())
	{
		change.first = 0.0;
		change.second = (obj2.Y() >= obj1.Y()) - (obj2.Y() < obj1.Y());
			
	}
	else if (obj1.Y() == obj2.Y())
	{
		change.first = (obj2.X() >= obj1.X()) - (obj2.X() < obj1.X());
		change.second = 0.0;
	}
	else
	{
		Coordinate diff = obj2 - obj1;
		Coordinate absDiff = Abs(diff);
		Coordinate direction = Sign(diff);
		if (absDiff.X() > absDiff.Y())
		{
			change.second = direction.Y();
			change.first = direction.X() * static_cast<double>(absDiff.X()) / absDiff.X();
		}
		else
		{
			change.first = direction.X();
			change.second = direction.Y() * static_cast<double>(absDiff.Y()) / absDiff.Y();
		}
	}
}

double DirectAttack::CalcDiversion(intVec & state, intVec & shelters, Coordinate & hit, int gridSize, Coordinate &  prevShotLocation, shootOutcomes & result) const
{
	Coordinate sides[4];
	for (size_t i = 0; i < 4; ++i)
		sides[i] = hit;

	++sides[0].X();
	--sides[1].X();
	++sides[2].Y();
	--sides[3].Y();

	std::vector<std::pair<double, int>> dist;
	for (size_t i = 0; i < 4; ++i)
	{
		if (sides[i] != prevShotLocation)
			dist.emplace_back(std::make_pair(sides[i].RealDistance(prevShotLocation), i));
	}

	std::sort(dist.begin(), dist.end());

	double outOfFrame = 0.0;
	for (size_t i = 0; i < NUM_DIVERSIONS; ++i)
	{
		intVec newState(state);
		double pDiverge = (1 - m_pHit) / NUM_DIVERSIONS;
		if ( InFrame(sides[dist[i].second], gridSize) )
		{
			int divLocation = sides[dist[i].second].X() + sides[dist[i].second].Y() * gridSize;
			auto v = newState.begin();
			for (; v != newState.end() ; ++v)
			{
				if (divLocation == *v)
				{
					if (!SearchForShelter(shelters, divLocation))
					{
						*v = gridSize * gridSize;
						result.emplace_back(std::make_pair(newState, pDiverge));
					}
					else
					{
						outOfFrame += pDiverge;
					}
					break;
				}
			}
			if (v == newState.end())
				outOfFrame += pDiverge;
		}
		else
			outOfFrame += pDiverge;
	}

	return outOfFrame;
}

bool DirectAttack::InFrame(Coordinate point, int gridSize)
{
	return (point.X() >= 0) & (point.X() < gridSize) & (point.Y() >= 0) & (point.Y() < gridSize);
}

bool DirectAttack::SearchForShelter(intVec shelters, int location)
{
	for (auto v : shelters)
	{
		if (v == location)
			return true;
	}

	return false;
}

