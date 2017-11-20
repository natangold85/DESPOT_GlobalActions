#include "Attack_Obj.h"



Attack_Obj::Attack_Obj(Coordinate& location, Move_Properties& movement, std::shared_ptr<Attack> attack)
: Movable_Obj(location, movement)
, m_attack(attack)
{
}

void Attack_Obj::AttackOnline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, double random) const
{
	// calculation of attack
	intVec prevState(state);
	m_attack->AttackOnline(objLoc, targetLoc, state, shelters, gridSize, random);
	
	// run on state and compare to previous state if other object than self object killed revive them
	for (int i = 1; i < state.size(); ++i)
	{
		state[i] = state[i] == gridSize * gridSize ? prevState[i] : state[i];
	}
};

void Attack_Obj::AttackOffline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, Attack::shootOutcomes & result) const
{
	// calculation of attack
	m_attack->AttackOffline(objLoc, targetLoc, state, shelters, gridSize, result);
};


