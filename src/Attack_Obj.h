#pragma once
#include <fstream>      // std::ofstream
#include <memory>      // unique_ptr
#include <vector>      // vector

#include "Movable_Obj.h"
#include "Move_Properties.h"
#include "Point.h"
#include "Attacks.h"

/// attack object on grid
class Attack_Obj : public Movable_Obj
{
	using intVec = std::vector<int>;
public:
	explicit Attack_Obj() = default;
	explicit Attack_Obj(Point& location, Move_Properties& movement, double attackRange, double pHit);
	virtual ~Attack_Obj() = default;
	Attack_Obj(const Attack_Obj&) = default;
	Attack_Obj& operator=(const Attack_Obj&) = default;

	bool CalcEnemyAttackDESPOT(int selfLoc, int objectLoc, intVec & otherObj, intVec & shelterLoc, int gridSize, double & random) const
											{ return m_attack->EnemyHitDESPOT(selfLoc, objectLoc, otherObj, shelterLoc, gridSize, random); };

	void CalcEnemyAttackSARSOP(intVec & state, intVec & shelters, int gridSize, double & pHit) const
											{ m_attack->EnemyHitSARSOP(state, shelters, gridSize, pHit); };

	///return if an object is in range of the attack rang
	bool IsInRange(int location, int otherObjLocation, int gridSize) const 
			{ return m_attack->IsInRange(location, otherObjLocation, gridSize); };
	///return the range of the attack
	double GetRange() const 
			{ return m_attack->GetRange(); };
	///return the prob to hit of the attack
	double GetPHit() const 
			{ return m_attack->GetPHit(); };
	///write object to file
	friend std::ofstream& operator<<(std::ofstream& out, const Attack_Obj& obj);
	///read object from file
	friend std::ifstream& operator>>(std::ifstream& in, Attack_Obj& obj);

protected:
	std::shared_ptr<Attack> m_attack;
};

