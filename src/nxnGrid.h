#ifndef NXNGRID_H
#define NXNGRID_H

#include <string>

#include "..\include\despot\core\pomdp.h"

#include <UDP_Prot.h>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

namespace despot 
{

/* =============================================================================
* NxNState class
* =============================================================================*/
/// the class is non-thread safe. 
/// the static members are specialized to one type of nxnGrid, so only one problem simulation can simultanuasely
class nxnGridState : public State 
{

public:
	using intVec = std::vector<int>;
	nxnGridState() = default;
	nxnGridState(STATE_TYPE state_id, double weight = 0.0);
	
	/// print the state
	std::string text() const;

	///update state given a new state
	void UpdateState(STATE_TYPE newStateIdx);
	void UpdateState(intVec newState);

	/// init static members so previous model will not affect current
	static void InitStatic();

	/// return the state vector given idx
	static void IdxToState(STATE_TYPE state, intVec & stateVec);
	/// return the state vector given observation
	//static void IdxToState(OBS_TYPE obs, intVec & stateVec) { IdxToState(static_cast<STATE_TYPE>(obs), stateVec); };
	/// return the state vector given state
	static void IdxToState(const State * state, intVec & stateVec) { IdxToState(state->state_id, stateVec); };
	/// return state given state vector
	static STATE_TYPE StateToIdx(const intVec & state);
	/// return state given state vector and grid size
	static STATE_TYPE StateToIdx(const intVec & state, int gridSize);

	static STATE_TYPE MaxState();

	/// return idx of object that exist in location -1 if the object does not exist
	static char ObjIdentity(intVec & state, int location);

	/// number of objects in grid (size of state vector)
	static int s_sizeState;
	/// the border between the enemies location and the non-involved location in the state vector
	static int s_numEnemies;
	/// gridSize
	static int s_gridSize;
	/// target location
	static int s_targetLoc;
	/// shelter vector
	static std::vector<int> s_shelters;
	
};

/* =============================================================================
* nxnGrid class
* =============================================================================*/
/// base class for nxnGrid model. derived classes are including step and actions implementations
/// the class is non thread safe. 
/// the static members are specialized to one type of nxnGrid so only one problem can simultaneously run
class nxnGrid : public DSPOMDP
{	
public:
	using intVec = std::vector<int>;
	using doubleVec = std::vector<double>;
	using lut_t = std::map < STATE_TYPE, doubleVec >;
	///	enum of objects
	enum OBJECT { SELF, ENEMY, NON_INV, SHELTER, TARGET, NUM_OBJECTS };
	enum VBS_OBJECTS { SELF_VBS, ENEMY_VBS, NON_INVOLVED_VBS, SHELTER_VBS, TARGET_VBS, OBSERVED_ENEMY_VBS, OBSERVED_NON_INVOLVED_VBS };
	enum MODEL_TYPE { OFFLINE, ONLINE, VBS };
	/// enum of type of calculation using sarsop data map
	enum CALCULATION_TYPE { WITHOUT, ALL, WO_NINV, JUST_ENEMY, ONE_ENEMY, WO_NINV_STUPID };
	 
	explicit nxnGrid(int gridSize, int traget, Self_Obj & self, std::vector<intVec> & objectsInitLoc);
	~nxnGrid() = default;
	
	// Functions for self use

	static MODEL_TYPE GetModelType() { return s_modelType; };

	/// init udp server
	static bool InitUDP(int portNum = -1);
	/// init offline decision lut
	static void InitLUT(lut_t & offlineLut, int offlineGridSize, MODEL_TYPE mType = ONLINE, CALCULATION_TYPE cType = WITHOUT);
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

	/// return a preferred action given model and prior
	static int ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward);
	/// return a preferred action given model and prior
	static int ChoosePreferredAction(const State * state, const DSPOMDP* m, double & expectedReward);

	/// initialize beliefState according to history
	void InitBeliefState(intVec & beliefState, const History & h) const;

	/// add to state shelter locations
	void AddSheltersLocations(intVec & state) const;
	
	/// recieve init state from simulator
	void InitState();
	/// send action to simulator
	void SendAction(int action);
	/// recieve current state from simulator and update rewrd and observation
	bool RcvState(State * s, int action, OBS_TYPE & obs, double & reward);

	// Functions for despot algorithm (for more information read the document "Tutorial on Using DESPOT with cpp model")

	/// return the probability for an observation given a state and an action
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const override;
	/// return the probability for an observation given a state and an action
	double ObsProbOneObj(OBS_TYPE obs, const State& state, int action, int objIdx) const;

	// create particle vector based on possible objects location and its weight
	void CreateParticleVec(std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles) const;

	/// return initial state
	virtual State *CreateStartState(std::string type) const override;
	///  return initial belief
	virtual Belief* InitialBelief(const State* start, std::string type) const override;
	
	/// initializ and allocate memory for a state
	virtual State* Allocate(STATE_TYPE state_id, double weight) const override;
	/// alocate memory and copy a state
	virtual State* Copy(const State* particle) const override;
	virtual void Free(State* particle) const override;
	virtual int NumActiveParticles() const override;

	/// return the max reward available
	virtual double GetMaxReward() const override{ return REWARD_WIN; };

	virtual void PrintState(const State& state, std::ostream& out = std::cout) const override;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const override;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const override;

/// functions that are necessary for step and action calculation
protected:
	/// check if 2 idx are in a given range on the grid
	static bool InRange(int idx1, int idx2, double range, int gridSize);

	/// return true if the robot is dead by enemy attack given random num(0-1). state is not reference by reason
	bool CalcIfDead(int enemyIdx, intVec state, double & randomNum) const;

	/// return identity of the objIdx
	enum OBJECT WhoAmI(int objIdx) const;

	/// create a vector of random numbers between 0 - 1
	static void CreateRandomVec(doubleVec & randomVec, int size);

	/// retrieve the observed state given current state and random number
	OBS_TYPE FindObservation(intVec & state, double p) const;

	/// advance the state to the next step position (regarding to other objects movement)
	void SetNextPosition(intVec & state, doubleVec & randomNum) const;

	/// change object location according to its movement properties and random number
	void CalcMovement(intVec & state, const Movable_Obj *object, double rand, int objIdx) const;

	// CHECK LOCATIONS :
	/// check if the next location (location + (x,y)) is in grid boundary
	bool InBoundary(int location, int xChange, int yChange) const;
	/// return true if the object idx location does not repeat in state
	static bool NoRepetitions(intVec & state, int currIdx, int gridSize);
	/// return true if location is valid for a given state (no repeats)
	static bool ValidLocation(intVec & state, int location);
	/// return true if location is valid for a given state (no repeats & in grid)
	static bool ValidLegalLocation(intVec & state, Coordinate location, int end, int gridSize);

private:
	/// implementation of choose prefferred action
	int ChoosePreferredActionIMP(intVec & state, double & expectedReward) const;

	/// create particle from available locations
	void CreateParticleVecRec(intVec & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const;

	/// create vector of all possible particles for init state
	void InitialBeliefStateRec(intVec & state, int currObj, double stateProb, std::vector<State*> & particles) const;

	/// find the observed state according to random number and original state
	void DecreasePObsRec(intVec & currState, const intVec & originalState, int currIdx, double pToDecrease, double &pLeft) const;

	/// return movement properties of an object
	const Move_Properties & GetMovement(int objIdx);
	
	/// move a specific object closer to robot
	void GetCloser(intVec & state, int objIdx, int gridSize) const;

	/// return move given random number (0-1)
	int FindObjMove(int currLocation, double random, int gridSize) const;

	/// return true if observedLocation is surrouning a location (in 1 of the 8 directions to location)
	static bool InSquare(int location, int location2, int squareSize, int gridSize);

	// Rescaling state functions:
	void ScaleState(const intVec & beliefState, intVec & scaledState) const;
	void ScaleState(const intVec & beliefState, intVec & scaledState, int newGridSize, int prevGridSize) const;

	static int FindMaxReward(const doubleVec & rewards1E, const doubleVec & rewards2E, double & maxReward);
	/// find max reward in reard vec and return its idx in the vector
	static int FindMaxReward(const doubleVec & rewards, double & maxReward);

	/// move non protected shelters to close non-object location
	void MoveNonProtectedShelters(const intVec & baliefState, intVec & scaledState, int newGridSize) const;
	/// move object location to the closest scaled spot but current scaled location
	void MoveObjectLocation(const intVec & beliefState, intVec & scaledState, int objIdx, int gridSize) const;
	/// in case self is on target after scaling we need to shift it from target
	void ShiftSelfFromTarget(const intVec & beliefState, intVec & scaledState, int gridSize) const;
	/// drop shelter from state (make shelter in accessible)
	void DropUnProtectedShelter(intVec & woShelter, int gridSize) const;

	/// return num enemies in calculation
	int NumEnemiesInCalc() const;
	/// return true if the lut is without non-involved
	int NumNonInvInCalc() const;


	/// init state according to s_objectsInitLocations
	void InitStateRandom();
	/// init state from vbs simulator
	void InitStateVBS();
	/// recieve from vbs smulator current state
	void InitStateIMP(intVec & state, intVec & observation);

	/// return the object location considering object type and object num(idx)
	int FindObject(intVec & state, intVec & identity, int object, int idx);
	int FindObject(intVec & state, intVec & identity, int object, int observedObj, bool & isObserved, int idx);

	/// add actions to specific enemy
	virtual void AddActionsToEnemy() = 0;
	virtual void AddActionsToShelter() = 0;

	/// return true if action is related to enemy
	virtual bool EnemyRelatedAction(int action) const = 0;
protected:
	int m_gridSize;
	int m_targetIdx;

	Self_Obj m_self;
	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_nonInvolvedVec;
	
	std::vector<ObjInGrid> m_shelters;

	/// for comunication with simulator
	static UDP_Server s_udpServer;

	// init locations of objects (needs to be in the size of number of objects - including self)
	static std::vector<intVec> s_objectsInitLocations;

	static int s_numBasicActions;
	static int s_numEnemyRelatedActions;

	/// offline data LUT
	static lut_t s_LUT;
	static int s_lutGridSize;
	/// type of model
	static enum MODEL_TYPE s_modelType;
	static enum CALCULATION_TYPE s_calculationType;

	/// rewards for different events
	static const double REWARD_WIN;
	static const double REWARD_LOSS;
	static const double REWARD_KILL_ENEMY;
	static const double REWARD_KILL_NINV;
	static const double REWARD_ILLEGAL_MOVE;

	// for model
	mutable MemoryPool<nxnGridState> memory_pool_;
};

} // end ns despot

#endif	// NXNGRID_H