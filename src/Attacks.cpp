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

inline DirectAttack::coord Abs(DirectAttack::coord num)
{
	num.first = Abs(num.first);
	num.second = Abs(num.second);
	return num;
}

inline DirectAttack::coord Sign(DirectAttack::coord num)
{
	num.first = Sign(num.first);
	num.second = Sign(num.second);
	return num;
}

inline DirectAttack::coord operator-(DirectAttack::coord & a, DirectAttack::coord & b)
{
	DirectAttack::coord ret(a);
	ret.first -= b.first;
	ret.second -= b.second;

	return ret;
}


inline bool operator!=(std::pair<double, double> & a, DirectAttack::coord & b)
{
	return (a.first != b.first) | (a.second != b.second);
}

inline std::pair<double, double> & operator+=(std::pair<double, double> & a, std::pair<double, double> & b)
{
	a.first += b.first;
	a.second += b.second;
	return a;
}

inline std::pair<double, double> operator+(std::pair<double, double> & a, std::pair<double, double> & b)
{
	DirectAttack::coord ret(a);
	ret.first += b.first;
	ret.second += b.second;

	return ret;
}

inline double Distance(std::pair<double, double> & a, std::pair<double, double> & b)
{
	std::pair<double, double> diff = std::make_pair(a.first - b.first, a.second - b.second);
	return sqrt(diff.first * diff.first + diff.second * diff.second);
}

inline double Distance(DirectAttack::coord & a, DirectAttack::coord & b)
{
	DirectAttack::coord diff = std::make_pair(a.first - b.first, a.second - b.second);
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

bool DirectAttack::EnemyHitDESPOT(int selfLoc, int attackerLoc, intVec & otherObj, intVec & shelterLoc, int gridSize, double & random) const
{
	coord self(selfLoc % gridSize, selfLoc / gridSize);
	coord enemy(attackerLoc % gridSize, attackerLoc / gridSize);

	double dist = Distance(self, enemy);

	bool isHit = false;

	if (dist <= m_range)
	{
		// check outcomes for enemy as attacker, don't treat cases when enemy kill non involved
		shootOutcomes result;
		intVec fightingObj{ 1, 0 };
		CalcAttackResult(otherObj, shelterLoc, fightingObj, gridSize, result);
		for (auto v : result)
		{
			random -= v.second;
			if (random <= 0.0)
			{
				otherObj = v.first;
				isHit = (otherObj[0] == gridSize * gridSize);
				break;
			}
		}
	}
	return isHit;
}

void DirectAttack::CalcSelfAttackDESPOT(intVec &state, intVec & shelters, int gridSize, double random) const
{
	// the attack is allways directed to enemy (location 1)
	intVec fightingObj{ 0, 1 };
	shootOutcomes result;
	CalcAttackResult(state, shelters, fightingObj, gridSize, result);

	for (auto v : result)
	{
		random -= v.second;
		if (random <= 0.0)
		{
			state = v.first;
			break;
		}
	}
}

void DirectAttack::EnemyHitSARSOP(intVec state, intVec shelters, int gridSize, double & pHit) const
{
	coord self(state[0] % gridSize, state[0] / gridSize);
	coord enemy(state[1] % gridSize, state[1] / gridSize);

	double dist = Distance(self, enemy);

	
	if (dist <= m_range)
	{
		// check outcomes for enemy as attacker, don't treat cases when enemy kill non involved
		shootOutcomes result;
		intVec fightingObj{ 1, 0 };
		CalcAttackResult(state, shelters, fightingObj, gridSize, result);
		pHit = 0.0;
		for (auto v : result)
		{
			if (v.first[0] == gridSize * gridSize)
				pHit += v.second;
		}
	}
	else
		pHit = 0.0;
}

void DirectAttack::CalcSelfAttackSARSOP(intVec & state, intVec & shelters, int gridSize, shootOutcomes & result) const
{
	// the attack is allways directed to enemy (location 1)
	intVec fightingObj{ 0, 1 };
	CalcAttackResult(state, shelters, fightingObj, gridSize, result);


	return;
}

bool DirectAttack::IsInRange(int location, int otherObjLocation, int gridSize) const
{
	coord self = std::make_pair(location % gridSize, location / gridSize);
	coord object = std::make_pair(otherObjLocation % gridSize, otherObjLocation / gridSize);

	return Distance(self, object) <= m_range;
}
void DirectAttack::CalcAttackResult(intVec state, intVec shelters, intVec & fightingObjects, int gridSize, shootOutcomes & result) const
{
	coord attacker(state[fightingObjects[0]] % gridSize, state[fightingObjects[0]] / gridSize);
	coord defender(state[fightingObjects[1]] % gridSize, state[fightingObjects[1]] / gridSize);

	std::pair<double, double> change;
	CalcChanges(attacker, defender, change);

	std::pair<double, double> selfLocation(attacker);
	std::pair<double, double> currLocation(attacker);
	coord prevLocation(attacker);
	coord location;

	while (prevLocation != defender)
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

void DirectAttack::CalcChanges(coord obj1, coord obj2, std::pair<double, double> & change)
{
	if (obj1.first == obj2.first)
	{
		change.first = 0.0;
		change.second = (obj2.second >= obj1.second) - (obj2.second < obj1.second);
			
	}
	else if (obj1.second == obj2.second)
	{
		change.first = (obj2.first >= obj1.first) - (obj2.first < obj1.first);
		change.second = 0.0;
	}
	else
	{
		coord diff = obj2 - obj1;
		coord absDiff = Abs(diff);
		coord direction = Sign(diff);
		if (absDiff.first > absDiff.second)
		{
			change.second = direction.second;
			change.first = direction.first * static_cast<double>(absDiff.first) / absDiff.first;
		}
		else
		{
			change.first = direction.first;
			change.second = direction.second * static_cast<double>(absDiff.second) / absDiff.first;
		}
	}
}

double DirectAttack::CalcDiversion(intVec & state, intVec & shelters, coord & hit, int gridSize, coord &  prevShotLocation, shootOutcomes & result) const
{
	coord sides[4];
	for (size_t i = 0; i < 4; ++i)
		sides[i] = hit;

	++sides[0].first;
	--sides[1].first;
	++sides[2].second;
	--sides[3].second;

	std::vector<std::pair<double, int>> dist;
	for (size_t i = 0; i < 4; ++i)
	{
		if (sides[i] != prevShotLocation)
			dist.emplace_back(std::make_pair(Distance(sides[i], prevShotLocation), i));
	}

	std::sort(dist.begin(), dist.end());

	double outOfFrame = 0.0;
	for (size_t i = 0; i < NUM_DIVERSIONS; ++i)
	{
		intVec newState(state);
		double pDiverge = (1 - m_pHit) / NUM_DIVERSIONS;
		if ( InFrame(sides[dist[i].second], gridSize) )
		{
			int divLocation = sides[dist[i].second].first + sides[dist[i].second].second * gridSize;
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

bool DirectAttack::InFrame(coord point, int gridSize)
{
	return (point.first >= 0) & (point.first < gridSize) & (point.second >= 0) & (point.second < gridSize);
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

