#pragma once
#include <string>
#include <vector>
/// basic attack for attack object
class Attack
{
public:
	using coord = std::pair<int, int>;
	using intVec = std::vector<int>;
	using stateAndProb = std::pair<intVec, double>;
	using shootOutcomes = std::vector<stateAndProb>;

	explicit Attack() = default;
	virtual ~Attack() = default;

	virtual bool EnemyHitDESPOT(int selfLoc, int attackerLoc, intVec & otherObj, intVec & shelterLoc, int gridSize, double & random) const = 0;
	virtual void CalcSelfAttackDESPOT(intVec & state, intVec & shelters, int gridSize, double random) const = 0;

	virtual void EnemyHitSARSOP(intVec state, intVec shelters, int gridSize, double & pHit) const = 0;
	virtual void CalcSelfAttackSARSOP(intVec & state, intVec & shelters, int gridSize, shootOutcomes & result) const = 0;
	
	virtual bool IsInRange(int location, int otheObjLocation, int gridSize) const = 0;
	virtual double GetRange() const = 0;
	virtual double GetPHit() const = 0;
};

class DirectAttack : public Attack
{
public:
	DirectAttack(double m_range, double pHit);
	~DirectAttack() = default;

	virtual bool EnemyHitDESPOT(int selfLoc, int attackerLoc, intVec & otherObj, intVec & shelterLoc, int gridSize, double & random) const;
	virtual void CalcSelfAttackDESPOT(intVec & state, intVec & shelters, int gridSize, double random) const;

	virtual void EnemyHitSARSOP(intVec state, intVec shelters, int gridSize, double & pHit) const;
	virtual void CalcSelfAttackSARSOP(intVec & state, intVec & shelters, int gridSize, shootOutcomes & result) const;

	virtual bool IsInRange(int location, int otheObjLocation, int gridSize) const;
	virtual double GetRange() const { return m_range; };
	virtual double GetPHit() const { return m_pHit; };

private:
	void CalcAttackResult(intVec state, intVec shelters, intVec & fightingObjects, int gridSize, shootOutcomes & result) const;
	static void CalcChanges(coord obj1, coord obj2, std::pair<double, double> & change);
	double CalcDiversion(intVec & state, intVec & shelters, coord & hit, int gridSize, coord & prevShotLocation, shootOutcomes & result) const;

	static bool InFrame(coord point, int gridSize);
	static bool SearchForShelter(intVec shelters, int location);

	double m_range;
	double m_pHit;
	
	static const int NUM_DIVERSIONS = 2;
};