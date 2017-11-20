#include "Self_Obj.h"

Self_Obj::Self_Obj(Coordinate& location, Move_Properties& movement, std::shared_ptr<Attack> attack, std::shared_ptr<Observation> observation)
: Attack_Obj(location, movement, attack)
, m_observation(observation)
{
}

void Self_Obj::AttackOnline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, double random) const
{
	m_attack->AttackOnline(objLoc, targetLoc, state, shelters, gridSize, random);
}

void Self_Obj::AttackOffline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, Attack::shootOutcomes & result) const
{
	m_attack->AttackOffline(objLoc, targetLoc, state, shelters, gridSize, result);
}