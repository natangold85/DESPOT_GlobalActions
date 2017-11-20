#include <fstream>      // std::ofstream
#include <memory>      // unique_ptr
#include <vector>      // vector

#include "Movable_Obj.h"
#include "Move_Properties.h"
#include "Coordinate.h"
#include "Attacks.h"

#ifndef ATTACK_OBJ_H
#define ATTACK_OBJ_H

/// attack object on grid
class Attack_Obj : public Movable_Obj
{
	using intVec = std::vector<int>;
public:
	explicit Attack_Obj() = default;
	explicit Attack_Obj(Coordinate& location, Move_Properties& movement, std::shared_ptr<Attack> attack);
	virtual ~Attack_Obj() = default;
	Attack_Obj(const Attack_Obj&) = default;
	Attack_Obj& operator=(const Attack_Obj&) = default;

	// calculations of attacks
	virtual void AttackOnline(int objLoc, int targetLoc, intVec & state, intVec & shelterLoc, int gridSize, double random) const;
	virtual void AttackOffline(int objLoc, int targetLoc, intVec & state, intVec & shelters, int gridSize, Attack::shootOutcomes & result) const;

	Attack *GetAttack() const  { return m_attack.get(); };

protected:
	std::shared_ptr<Attack> m_attack;
};

# endif //ATTACK_OBJ_H