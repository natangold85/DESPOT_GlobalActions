#pragma once
#include <fstream>      // std::ofstream
#include <vector>      // vector
#include <string>      // string

#include "Attack_Obj.h"
#include "Move_Properties.h"
#include "Point.h"

/// class for self object in nxnGrid
class Self_Obj :
	public Attack_Obj
{
public:
	using intVec = std::vector<int>;
	using shootOutcomes = std::vector<std::pair<intVec, double>>;

	explicit Self_Obj() = default;
	explicit Self_Obj(Point& location, Move_Properties& movement, double attackRange, double pHit, int rangeObs, double pObservation);
	virtual ~Self_Obj() = default;
	Self_Obj(const Self_Obj&) = default;

	double GetSelfPStay() const { return GetMovement().GetStay(); };
	double GetSelfPMove() const { return GetMovement().GetToward(); };

	void CalcSelfAttackDESPOT(intVec & state, intVec & shelters, int gridSize, double & random) const
										{ m_attack->CalcSelfAttackDESPOT(state, shelters, gridSize, random);};

	void CalcSelfAttackSARSOP(intVec state, intVec & shelters, int gridSize, shootOutcomes & result) const
										{ m_attack->CalcSelfAttackSARSOP(state, shelters, gridSize, result); };

	/// return the p for successful observation when the object in range
	double GetPObs() const { return m_pObservation; }
	/// return the range of the observation
	int GetRangeObs() const { return m_rangeObs; }

	/// write to file
	friend std::ofstream& operator<<(std::ofstream& in, const Self_Obj& obj);
	/// read from file
	friend std::ifstream& operator>>(std::ifstream& in, Self_Obj& obj);

private:
	int m_rangeObs;
	double m_pObservation;
};