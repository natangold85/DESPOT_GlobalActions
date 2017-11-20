#ifndef NXNGRID_GLOBALACTIONS_H
#define NXNGRID_GLOBALACTIONS_H

#pragma once
#include <string>

#include "..\include\despot\core\pomdp.h"

#include "nxnGrid.h"
#include "Coordinate.h"

namespace despot 
{

/* =============================================================================
* Global Actions class
* =============================================================================*/
/// the class is non thread safe. 
/// the static members are specialized to one type of nxnGrid so only one problem can simultaneously run
class nxnGridGlobalActions : public nxnGrid
{	
public:
	enum ACTION { MOVE_TO_TARGET, MOVE_TO_SHELTER};
	enum ENEMY_ACTION { ATTACK, MOVE_FROM_ENEMY, NUM_ENEMY_ACTIONS };

	explicit nxnGridGlobalActions(int gridSize, int traget, Self_Obj & self, std::vector<intVec> & objectsInitLoc, bool isMoveFromEnemyExist = true);
	~nxnGridGlobalActions() = default;
	
	// Functions for self use

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, OBS_TYPE lastObs, double & reward, OBS_TYPE& obs) const override;

	
	virtual int NumActions() const override;

	/// return the min reward valued action (needed for the despot algorithm)
	virtual ValuedAction GetMinRewardAction() const override;
	virtual void PrintAction(int action, std::ostream& out = std::cout) const  override;

private:

	// add actions to specific enemy
	virtual void AddActionsToEnemy() override;
	virtual void AddActionsToShelter() override;

	// ACTIONS FUNCTION:
	bool MoveToTarget(intVec & state, double random) const;
	bool MoveToShelter(intVec & state, double random) const;
	void Attack(intVec & state, int idxEnemy, double random, OBS_TYPE lastObs) const;
	void MoveFromEnemy(intVec & state, int idxEnemy, double random, OBS_TYPE lastObs) const;

	/// try move to goTo (depend on random number)
	bool MoveToLocation(intVec & state, Coordinate & goTo, double random) const;
	/// implementation of MoveToLocation
	int MoveToLocationIMP(intVec & state, Coordinate & goTo) const;

	/// return the farthest available location from goFrom
	int MoveFromLocation(intVec & state, Coordinate & goFrom) const;

	/// return the nearest shelter location
	int NearestShelter(int loc) const;

	/// return num actions that not related to enemies
	int NumBasicActions() const;
	/// return num actions that related to enemies
	int NumEnemyActions() const;

	/// return true if the action is enemy related action
	virtual bool EnemyRelatedAction(int action) const override;
};

} // end ns despot

#endif	// NXNGRID_GLOBALACTIONS_H