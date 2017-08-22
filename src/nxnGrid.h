#ifndef NXNGRID_H
#define NXNGRID_H

#pragma once
#include <string>

#include "..\include\despot\core\pomdp.h"


#include "..\CreateLUT\StateActionLUT.h" // lut
#include "MapOfPomdp.h"

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

namespace despot 
{

using state_t = std::vector<int>;
/* =============================================================================
* NxNState class
* =============================================================================*/
/// the class is non-thread safe. 
/// the static members are specialized to one type of nxnGrid, so only one problem simulation can simultanuasely
class nxnGridState : public State 
{
public:

	nxnGridState() = default;
	nxnGridState(int state_id, double weight = 0.0);
	
	/// print the state
	std::string text() const;

	///update state given a new state
	void UpdateState(int newStateIdx);
	void UpdateState(state_t newState);

	/// return the state vector given idx
	static state_t IdxToState(int state);
	/// return the state vector given state
	static state_t IdxToState(nxnGridState state) { return IdxToState(state.state_id); };
	/// return state given state vector
	static int StateToIdx(state_t & state);

	static int MaxState();
	/// return true if enemy is located in a given location
	bool EnemyLoc(state_t & state, int location) const;
	/// return true if non-involved object is located in a given location
	bool NInvLoc(state_t & state, int location) const;
	/// return true if shelter is located in a given location
	bool ShelterLoc(int location) const;

	/// number of objects in grid (size of state vector)
	static int s_sizeState;
	/// the border between the enemies location and the non-involved location in the state vector
	static int s_endEnemyIdx;
	/// gridSize
	static int s_gridSize;
	/// target location
	static int s_targetIdx;
	/// shelter vector
	static std::vector<ObjInGrid> s_shelters;
	
};

/* =============================================================================
* nxnGrid class
* =============================================================================*/
/// the class is non thread safe. 
/// the static members are specialized to one type of nxnGrid so only one problem can simultaneously run
class nxnGrid : public DSPOMDP
{
	using coord = std::pair<int, int>;
private:
	mutable MemoryPool<nxnGridState> memory_pool_;
public:
	///	enum of all actions
	enum ACTION { MOVE_TO_TARGET, MOVE_TO_SHELTER, ATTACK, MOVE_FROM_ENEMY, NUM_ACTIONS };
	///	enum of objects
	enum OBJECT { SELF, ENEMY, NON_INV, SHELTER, TARGET};
	/// enum of type of calculation using sarsop data map
	enum CALCULATION_TYPE { WITHOUT, ALL, WO_NINV, JUST_ENEMY, STUPID};
	 
	explicit nxnGrid(int gridSize, int traget, Self_Obj & self, std::shared_ptr<StateActionLUT> lut, enum CALCULATION_TYPE calcType = ALL);
	~nxnGrid() = default;
	
	// Functions for self use

	/// add enemy object
	void AddObj(Attack_Obj&& obj); 
	/// add non involved object
	void AddObj(Movable_Obj&& obj);
	/// add shelter
	void AddObj(ObjInGrid&& obj);

	/// count number of moving objects(self, enemies, non involved)
	int CountMovingObjects() const;

	/// return grid size
	int GetGridSize() const { return m_gridSize; };

	/// get observed location of object (identified by idx) given observation
	int GetObsLoc(OBS_TYPE obs, int objIdx) const;

	/// return true if an observed location of an object is in observation range
	bool InObsRange(OBS_TYPE obs, int objIdx) const;

	/// return probability for successful observation
	double GetPObs() const { return m_self.GetPObs(); }

	/// return a preferred action given model and prior
	static int ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward);

	/// initialize beliefState according to history
	void InitBeliefState(state_t & beliefState, const History & h) const;

	/// create a random init state for the model
	void RandInitState();

	// Functions for despot algorithm (for more information read the document "Tutorial on Using DESPOT with cpp model")

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, double & reward, OBS_TYPE& obs) const;

	virtual int NumStates() const;
	virtual int NumActions() const { return NUM_ACTIONS; };

	/// return the probability for an observation given a state and an action
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	/// return initial state
	virtual State *CreateStartState(std::string type) const;
	///  return initial belief
	virtual Belief* InitialBelief(const State* start, std::string type) const;
	
	/// initializ and allocate memory for a state
	virtual State* Allocate(int state_id, double weight) const;
	/// alocate memory and copy a state
	virtual State* Copy(const State* particle) const;
	virtual void Free(State* particle) const;
	virtual int NumActiveParticles() const;

	/// return the min reward valued action (needed for the despot algorithm)
	virtual ValuedAction GetMinRewardAction() const { return ValuedAction(MOVE_FROM_ENEMY, -10.0); }
	/// return the max reward available
	virtual double GetMaxReward() const { return REWARD_WIN; };

	// POSSIBLE IMPROVEMENT OF MODEL

	/// create upper bound particle (for more information read the document "Tutorial on Using DESPOT with cpp model")
	//virtual ParticleUpperBound* CreateParticleUpperBound(std::string name = "DEFAULT") const;
	/// create upper bound scenario (for more information read the document "Tutorial on Using DESPOT with cpp model")
	//virtual ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
	//	std::string particle_bound_name = "DEFAULT") const;
	
	/// create upper bound belief (not implemented yet)
	//virtual BeliefUpperBound* CreateBeliefUpperBound(std::string name = "DEFAULT") const;

	/// create lower bound scenario (for more information read the document "Tutorial on Using DESPOT with cpp model")
	//virtual ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;
	
	/// create lower bound belief (not implemented yet)
	//virtual BeliefLowerBound* CreateBeliefLowerBound(std::string name = "DEFAULT") const;

	virtual void PrintState(const State& state, std::ostream& out = std::cout) const;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	virtual void PrintAction(int action, std::ostream& out = std::cout) const;


private:
	// actions functions
	
	void MoveToTarget(state_t & state, double random) const;
	void MoveToShelter(state_t & state, double random) const;
	void Attack(state_t & state, double random) const;
	void MoveFromEnemy(state_t & state, double random) const;

	/// try move to goTo (depend on random number)
	void MoveToLocation(state_t & state, std::pair<int, int> & goTo, double random) const;
	/// implementation of MoveToLocation
	int MoveToLocationIMP(state_t & state, std::pair<int, int> & goTo) const;

	/// return the farthest available location from goFrom
	int MoveFromLocation(state_t & state, std::pair<int, int> & goFrom) const;

	/// check if 2 idx are in a given range on the grid
	static bool InRangeAttack(int idx1, int idx2, double range, int gridSize);

	/// return true if the object are in range or dead
	static bool InRangeObservation(int idx1, int idx2, double range, int gridSize);

	/// return true if the robot is dead by enemy attack given random num(0-1). state is not reference by reason
	bool CalcIfDead(int enemyIdx, state_t state, double & randomNum) const;

	/// return identity of the objIdx
	enum OBJECT WhoAmI(int objIdx) const;

	/// create particles for belief state vector
	void InsertParticlesRec(state_t & state, std::vector<State *> & particles, double pToBelief, int currIdx) const;
	
	/// create a vector of random numbers between 0 - 1
	static std::vector<double> CreateRandomVec(int size);

	/// retrieve the observed state given current state and random number
	int FindObservation(state_t state, double p) const;
	
	/// advance the state to the next step position (regarding to other objects movement)
	void SetNextPosition(state_t & state, std::vector<double> & randomNum) const;
	
	/// change object location according to its movement properties and random number
	void CalcMovement(state_t & state, const Movable_Obj *object, double rand, int objIdx) const;

	/// find the observed state according to random number and original state
	void DecreasePObsRec(state_t & currState, state_t & originalState, int currIdx, double pToDecrease, double &pLeft) const;

	/// return movement properties of an object
	const Move_Properties & GetMovement(int objIdx);
	
	/// move a specific object closer to robot
	void GetCloser(state_t & state, int objIdx, int gridSize) const;

	/// return move given random number (0-1)
	int FindObjMove(int currLocation, double random, int gridSize) const;

	/// return true if observedLocation is surrouning a location (in 1 of the 8 directions to location)
	static bool IsNear(int location, int observedLocation, int gridSize);

	// CHECK LOCATIONS :
	/// check if the next location (location + (x,y)) is in grid boundary
	bool InBoundary(int location, int xChange, int yChange) const;
	/// return true if the object idx location does not repeat in state
	static bool NoRepetitions(state_t & state, int currIdx, int gridSize);
	/// return true if location is valid for a given state (no repeats)
	static bool ValidLocation(state_t & state, int location);
	/// return true if location is valid for a given state (no repeats & in grid)
	static bool ValidLegalLocation(state_t & state, int location, int gridSize);

private:
	int m_gridSize;
	int m_targetIdx;

	Self_Obj m_self;
	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_nInvolvedVec;
	
	std::vector<ObjInGrid> m_shelters;

	/// offline data LUT
	std::shared_ptr<StateActionLUT> m_LUT;

	/// rewards for different events
	static const double REWARD_WIN;
	static const double REWARD_LOSS;
	static const double REWARD_KILL_ENEMY;
	static const double REWARD_KILL_NINV;

	/// type of calculation using sarsop data
	static enum CALCULATION_TYPE s_calculationType;
};

} // end ns despot

#endif	// MAPOFPOMDP_H