//	Purpose: create POMDP format given specific state (including 1 robot, enemies, non-involved movable objects and shelters on NxN Grid)
//			win is getting to an idx in the map(idxTarget), loss is killing non-involved and the death of the robot by the enemies.

//	Author: Natan Gold

//	COMMENTS REGARDING THE RULES OF THE GAME:
//	1-	moving objects can not be in the same idx in the grid
//	2-	a shot of the robot is absorbed by the first object that it his met whether it is an enemy, non-involved or shelter
//	3-	when in enemy range the robot has specific probability to be killed by the enemy
//	4-	there should not be enemy or self with range 0
//  5-	the model is yet not support varied initial locations of objects
//	6-	the model is yet not support more than 1 enemy and 1 shelter


#pragma once

#include <vector>
#include <map>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

class POMDP_Writer
{
	/// operator << to print the pomdp
	friend std::ostream& operator<<(std::ostream& o, const POMDP_Writer& map);
	/// stream pomdp to file
	friend std::ofstream& operator<<(std::ofstream& out, const POMDP_Writer& map);
	/// read pomdp from file
	friend std::ifstream& operator>>(std::ifstream& in, POMDP_Writer& map);

public:
	explicit POMDP_Writer() = default;
	explicit POMDP_Writer(int gridSize, int targetIdx, Self_Obj& self, double discount = 0.95);
	~POMDP_Writer() = default;
	POMDP_Writer(const POMDP_Writer &) = default;
	POMDP_Writer& operator=(const POMDP_Writer&) = default;

	using intVec = std::vector<int>;
	using mapProb = std::map<intVec, double>;
	using pairMap = std::pair<intVec, double>;

	enum IDENTITY { SELF = 0, ENEMY = 1, NON_INVOLVED = 2, SHELTER = 3 };

	/// save model in pomdp format to file
	void SaveInFormat(FILE *fptr);

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

	// change model functions

	void SetLocationSelf(Coordinate & newLocation);
	void SetLocationEnemy(Coordinate & newLocation, int idxObj);
	void SetLocationNonInv(Coordinate & newLocation, int idxObj);
	void SetLocationShelter(Coordinate & newLocation, int idxObj);
	void SetTarget(int idx);
	void SetGridSize(int gridSize);

private:
	// map & model properties
	int m_gridSize;
	int m_targetIdx;
	double m_discount;

	// objects in model
	Self_Obj m_self;
	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_nonInvolvedVec;
	std::vector<ObjInGrid> m_shelterVec;

	// main functions for saving format to file: 
	
	/// insert init lines, init states, state list and comments to file
	void CommentsAndInitLines(FILE *fptr);
	/// insert transitions of all actions to file
	void AddAllActions(FILE *fptr);
	/// insert observation and reward to file
	void ObservationsAndRewards(FILE *fptr);


	/// insert of possible state or observations(depending on type) to buffer
	void CalcStatesAndObs(const char * type, std::string& buffer);
	/// run on all possible states or observations and insert them to buffer
	void CalcS_ORec(intVec& state, int currIdx, const char * type, std::string& buffer);

	/// insert probability to init of all states
	void CalcStartState(std::string& buffer);
	/// run on all states and insert the probability of each state to init in
	void CalcStartStateRec(intVec& state, intVec& initState, int currIdx, std::string& buffer);

	// Calculation of actions result

	/// insert states where robot is in target position transition to win state to buffer
	void AddTargetPositionRec(intVec & state, int currIdx ,std::string& buffer); 
	/// calculation of transition of all actions
	void AddActionsAllStates(std::string& buffer);	
	/// insert tarnsition to init state from win/loss
	void StartAgain(std::string& buffer, const std::string& terminatingState);

	/// run on all possible states and calculate the end-state fro all actions
	void AddActionsRec(intVec & state, intVec & shelters, int currObj, std::string & buffer);

	/// add action attack with state and shelters to buffer
	void AddAttack(intVec & state, intVec & shelters, std::string & buffer) const;
	/// add action move to target with state and shelters to buffer
	void AddMoveToTarget(intVec & state, intVec & shelters, std::string & buffer) const;
	/// add action move to shelter with state and shelters to buffer
	void AddMoveToShelter(intVec & state, intVec & shelters, std::string & buffer) const;
	/// add action move from enemy with state and shelters to buffer
	void AddMoveFromEnemy(intVec & state, intVec & shelters, std::string & buffer) const;

	/// move to a specific location return the peobability to loss in that action
	double MoveToLocation(intVec & state, intVec & shelters, int location, std::string & action, std::string & buffer) const;
	/// move to location return new self location (-1 if there is no way to get closer to location)
	int MoveToLocationIMP(intVec & state, int goTo) const;
	/// return the farthest point reachable of self from a specific location
	int MoveFromLocation(intVec & state, int location) const;

	/// insert the end-states positions (states and probabilities) from a single state(state) to buffer
	double PositionSingleState(intVec& state, intVec& currentState, intVec & shelters, std::string& action, std::string& buffer) const;

	/// create location vector of all shelters
	void CreateShleterVec(intVec & shelters) const;
	/// return location of the nearest shelter
	int FindNearestShelter(int location) const;
	/// return true if there is dead non-involved
	bool IsNonInvDead(intVec & state) const;
	
	/// calculate possible move states from a start-state
	void CalcMoveStates(intVec & state, std::vector<intVec> & moveStates) const;
	/// insert possible moveStates (states and probability) to pMap
	void AddMoveStatesRec(intVec & state, std::vector<intVec> & moveStates, intVec & arrOfIdx, int currIdx, mapProb & pMap) const;

	/// add state and probability to state (itr) to buffer
	static void AddStateToBuffer(std::string& buffer, pairMap itr, int gridSize);
	
	/// translate a state to the pomdp format string
	std::string GetStringState(intVec& state) const;
	/// translate a state to the pomdp format string with a different initialize char
	std::string GetStringState(intVec& state, const char * type) const;

	/// returns the real end-state from a given moveState
	intVec MoveToIdx(intVec stateVec, std::vector<intVec> & moveStates, intVec & arrOfIdx) const;
	/// calculate probability to move for a given moveState
	double CalcProb2Move(intVec & arrOfIdx) const;

	/// returns true if the location is sheltered
	bool SearchForShelter(int location) const;

	///insert all observations to buffer
	void CalcObs(std::string& buffer);
	/// run on all states and insert observation probability to buffer
	void CalcObsRec(intVec& state, int currIdx, std::string& buffer);
	/// insert observation probability of a single state
	void CalcObsSingleState(intVec& state, std::string& buffer);
	/// calculate probability of observation from originalState
	void CalcObsMapRec(intVec& state, intVec& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, int currIdx);
	/// run on 8 close locations and diverge observation probability to those locations
	void DivergeObs(intVec& state, intVec& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, int currIdx, bool isPrevRange);
	/// return true if object is in observation range
	static bool InObsRange(int self, int object, int gridSize, int range);
	
	// search for repetition or legal locations in a state or moveState.
	
	/// return true if state + advance factor is inside the grid
	static bool InBoundary(int state, int advanceFactor, int gridSize);
	/// return true if state + xChange + yChange is inside the grid
	static bool InBoundary(int state, int xChange, int yChange, int gridSize);

	/// return false until the state is not legit state and make a single correction of it (bring the non-legal location to previous location using moveStates )
	bool NoRepetitionCheckAndCorrect(intVec& state, std::vector<intVec> & moveStates, intVec & arrOfIdx) const;
	/// return true if location is not presence on state
	bool NoRepeatsLocation(intVec& state, int location) const;
	/// return true if there is no repetitions in all state
	bool NoRepeatsAll(intVec& state) const;
	/// return true if the state idx is with no repetitions to previous objects locations
	bool NoRepeats(intVec& state, int idx) const;

	/// return true if objIdx is an idx of an enemy
	bool IsEnemy(int objIdx) const;
	/// return true if there is any deads in state
	bool AnyDead(intVec& state) const;
};

