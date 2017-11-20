//	Purpose: create POMDP format given specific state (including 1 robot, enemies, non-involved movable objects and shelters on NxN Grid)
//			win is getting to an idx in the map(idxTarget), loss is killing non-involved and the death of the robot by the enemies.

//	Author: Natan Gold

//	COMMENTS REGARDING THE RULES OF THE GAME:
//	1-	moving objects can not be in the same idx in the grid
//	2-	a shot of the robot is absorbed by the first object that it met whether it is an enemy, non-involved or shelter
//	3-	when in enemy range the robot has specific probability to be killed by the enemy
//	4-	there should not be enemy or self with range 0
//  5-	the model is yet not support varied initial locations of objects
//	6-	the model is yet not support more than 1 enemy and 1 shelter


#ifndef NXNGRIDOFFLINE_H
#define NXNGRIDOFFLINE_H

#include <vector>
#include <map>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

class nxnGridOffline
{
	/// operator << to print the pomdp
	friend std::ostream& operator<<(std::ostream& o, const nxnGridOffline& map);
	/// stream pomdp to file
	friend std::ofstream& operator<<(std::ofstream& out, const nxnGridOffline& map);
	/// read pomdp from file
	friend std::ifstream& operator>>(std::ifstream& in, nxnGridOffline& map);

public:
	explicit nxnGridOffline() = default;
	explicit nxnGridOffline(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount = 0.95);
	~nxnGridOffline() = default;
	nxnGridOffline(const nxnGridOffline &) = default;
	nxnGridOffline& operator=(const nxnGridOffline&) = default;

	using intVec = std::vector<int>;
	using mapProb = std::map<intVec, double>;
	using pairMap = std::pair<intVec, double>;

	enum IDENTITY { SELF = 0, ENEMY = 1, NON_INVOLVED = 2, SHELTER = 3 };

	/// save model in pomdp format to file
	virtual void SaveInPomdpFormat(FILE *fptr) = 0;

	// init model functions

	void UpdateSelf(Self_Obj && obj);
	void AddObj(Attack_Obj&& obj);
	void AddObj(Movable_Obj&& obj);
	void AddObj(ObjInGrid&& obj);

	// count objects functions

	/// return number of moving objects in grid
	int CountMovableObj() const;
	
	int CountEnemies() const;
	int CountNInv() const;
	int CountShelters() const;
	int GetGridSize() const;
	bool IsFullyObs() const { return m_isFullyObs; };
	// change model functions

	void SetLocationSelf(Coordinate & newLocation);
	void SetLocationEnemy(Coordinate & newLocation, int idxObj);
	void SetLocationNonInv(Coordinate & newLocation, int idxObj);
	void SetLocationShelter(Coordinate & newLocation, int idxObj);
	void SetTarget(int idx);
	void SetGridSize(int gridSize);

	long StateCount2StateIdx(long stateCount) const;
	/// return state given stateIdx
	intVec Idx2State(long stateIdx);

	virtual int GetNumActions() const = 0;
/// protected to access for actions implementations
protected:
	// map & model properties
	int m_gridSize;
	int m_targetIdx;
	double m_discount;
	bool m_isFullyObs;
	// objects in model
	Self_Obj m_self;
	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_nonInvolvedVec;
	std::vector<ObjInGrid> m_shelterVec;

	/// TODO: implement this variable in function arguments
	/// to convey probability between calculations
	static double s_pLeftProbability;

/// functions necessary for actions implementations

	// main functions for saving format to file: 

	/// insert observation and reward to file
	void ObservationsAndRewards(FILE *fptr);

	/// insert of possible state or observations(depending on type) to buffer
	void CalcStatesAndObs(const char * type, std::string& buffer);

	/// insert probability to init of all states
	void CalcStartState(std::string& buffer);

	/// insert the end-states positions (states and probabilities) from a single state(state) to buffer
	double PositionSingleState(intVec& state, intVec& currentState, intVec & shelters, std::string& action, std::string& buffer) const;
	
	/// insert states where robot is in target position transition to win state to buffer
	void AddTargetPositionRec(intVec & state, int currIdx, std::string& buffer);
	/// calculation of transition of all actions
	void AddActionsAllStates(std::string& buffer);

	/// return true if there is dead non-involved
	bool IsNonInvDead(intVec & state) const;

	/// returns true if the location is sheltered
	bool SearchForShelter(int location) const;
	/// return location of the nearest shelter
	int FindNearestShelter(int location) const;
	/// create location vector of all shelters
	void CreateShleterVec(intVec & shelters) const;

	/// translate a state to the pomdp format string
	std::string GetStringState(intVec& state) const;
	/// translate a state to the pomdp format string with a different initialize char
	std::string GetStringState(intVec& state, const char * type) const;
	// translate to string with higher precision
	static std::string Dbl2Str(double d);

	/// return true if location is not presence on state
	bool NoRepeatsLocation(intVec& state, int location) const;
	/// return true if the state idx is with no repetitions to previous objects locations
	bool NoRepeats(intVec& state, int idx) const;
	/// return true if state + advance factor is inside the grid
	static bool InBoundary(int state, int advanceFactor, int gridSize);
	/// return true if state + xChange + yChange is inside the grid
	static bool InBoundary(int state, int xChange, int yChange, int gridSize);

	/// return true if objIdx is an idx of an enemy
	bool IsEnemy(int objIdx) const;
private:

	// main functions for saving format to file: 

	/// insert init lines, init states, state list and comments to file
	virtual void CommentsAndInitLines(FILE *fptr) = 0;
	/// insert transitions of all actions to file
	virtual void AddAllActions(FILE *fptr) = 0;

	/// find corresponding state to stateCount (basic count of state)
	void FindStateCount(intVec & state, long & count, int currIdx) const;
	/// return state_id given state and gridSize
	static long State2Idx(intVec & state, int gridSize);


	/// run on all possible states or observations and insert them to buffer
	void CalcS_ORec(intVec& state, int currIdx, const char * type, std::string& buffer);

	/// run on all states and insert the probability of each state to init in
	void CalcStartStateRec(intVec& state, intVec& initState, int currIdx, std::string& buffer);

	
	/// calculate possible move states from a start-state
	void CalcMoveStates(intVec & state, std::vector<intVec> & moveStates) const;
	/// insert possible moveStates (states and probability) to pMap
	void AddMoveStatesRec(intVec & state, std::vector<intVec> & moveStates, intVec & arrOfIdx, int currIdx, mapProb & pMap) const;

	/// add state and probability to state (itr) to buffer
	static void AddStateToBuffer(std::string& buffer, pairMap & itr, int gridSize);

	/// returns the real end-state from a given moveState
	intVec MoveToIdx(intVec stateVec, std::vector<intVec> & moveStates, intVec arrOfIdx) const;
	/// calculate probability to move for a given moveState
	double CalcProb2Move(const intVec & arrOfIdx) const;

	///insert all observations to buffer
	void CalcObs(std::string& buffer);
	/// run on all states and insert observation probability to buffer
	void CalcObsRec(intVec& state, int currIdx, std::string& buffer);
	/// insert observation probability of a single state
	void CalcObsSingleState(intVec& state, std::string& buffer);
	/// calculate probability of observation from originalState
	void CalcObsMapRec(intVec& state, intVec& originalState, mapProb& pMap, double pCurr, int currObj);
	/// run on 8 close locations and diverge observation probability to those locations
	void DivergeObsFirstSquare(intVec& state, intVec& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, int currIdx);
	/// run on 16 close locations and diverge observation probability to those locations
	void DivergeObsSecondSquare(intVec& state, intVec& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, int currIdx);

	/// return true if object is in observation range
	static bool InObsRange(int self, int object, int gridSize, int range);
	
	// search for repetition or legal locations in a state or moveState.

	/// return false until the state is not legit state and make a single correction of it (bring the non-legal location to previous location using moveStates )
	bool NoRepetitionCheckAndCorrect(intVec& state, std::vector<intVec> & moveStates, intVec & arrOfIdx) const;
	/// return true if there is no repetitions in all state
	bool NoRepeatsAll(intVec& state) const;

	/// return true if there is any deads in state
	bool AnyDead(intVec& state) const;
};

# endif //NXNGRIDOFFLINE_H