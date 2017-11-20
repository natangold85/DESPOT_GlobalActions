#ifndef NXNGRID_LOCALACTIONS_H
#define NXNGRID_LOCALACTIONS_H

#pragma once
#include <string>

#include "..\include\despot\core\pomdp.h"

#include "nxnGrid.h"
#include "Self_Obj.h"
#include "Coordinate.h"

namespace despot 
{

/* =============================================================================
* Global Actions class
* =============================================================================*/
/// the class is non thread safe. 
/// the static members are specialized to one type of nxnGrid so only one problem can simultaneously run
class nxnGridLocalActions : public nxnGrid
{	
public:
	///	enum of all actions
	enum ACTION { STAY, NORTH, SOUTH, EAST, WEST, NORTH_EAST, NORTH_WEST, SOUTH_EAST, SOUTH_WEST, NUM_BASIC_ACTIONS };

	explicit nxnGridLocalActions(int gridSize, int traget, Self_Obj & self, std::vector<intVec> & objectsInitLoc);
	~nxnGridLocalActions() = default;
	
	// Functions for self use

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, OBS_TYPE lastObs, double & reward, OBS_TYPE& obs) const override;

	virtual int NumActions() const override;

	/// return the min reward valued action (needed for the despot algorithm)
	virtual ValuedAction GetMinRewardAction() const override { return ValuedAction(STAY, -10.0); }
	virtual void PrintAction(int action, std::ostream& out = std::cout) const override;


private:
	// add actions to specific enemy
	virtual void AddActionsToEnemy() override;
	virtual void AddActionsToShelter() override;

	// ACTIONS FUNCTION:
	/// make move and update state return true if the move is valid (regardless if the move was successful or not) 
	bool MakeMove(intVec & state, double random, ACTION action) const;
	/// make attack and update state
	void Attack(intVec & state, int enemyIdx, double random, OBS_TYPE lastObs) const;

	/// try move to goTo (depend on random number)
	void MoveToLocation(intVec & state, Coordinate & goTo, double random) const;
	/// implementation of MoveToLocation
	int MoveToLocationIMP(intVec & state, Coordinate & goTo) const;

	virtual bool EnemyRelatedAction(int action) const override;
};

} // end ns despot

#endif	// NXNGRID_LOCALACTIONS_H