#pragma once

#include <vector>
#include <map>
#include "../src/MapOfPomdp.h"
#include "../src/Coordinate.h"

class DataForObjects
{
public:
	double m_moveProb;
	double m_pHit;
	double m_pHitEnemy;
	double m_attackRange;
	double m_attackRangeEnemy;
	int m_observationRange;
	double m_pObservation;
};

class StateActionLUT
{
public:
	using intVec = std::vector<int>;
	using actionReward = std::pair<int, double>;
	using stateRewardMap = std::map<int, double>;
	using stateActionMap = std::map<int, actionReward>;

	///	enum of all actions
	enum ACTION { MOVE_TO_TARGET, MOVE_TO_SHELTER, ATTACK, MOVE_FROM_ENEMY, NUM_ACTIONS };

	///	enum of all actions
	enum LUT_TYPE { SINGLE, AVG, MAX, MIN };

	/// enum of key idx
	enum KEY_IDX { GRIDSIZE, NUM_ENEMIES, NUM_NON_INVOLVED, NUM_SHELTERS, KEYSIZE };

	///	enum of moving objects
	enum OBJECT { SELF, ENEMY, NON_INVOLVED, SHELTER };

	///  ctor to read and use exist lut
	explicit StateActionLUT(int despotGridSize);
	/// ctor to create a new lut
	explicit StateActionLUT(std::vector<intVec> & keys, int onlineGridSize, LUT_TYPE type = SINGLE);
	~StateActionLUT() = default;

	/// create lut using offline data(stateReward) with offline grid size(gridSize) and given data on objects (dataForObjects - ranges are offline ranges)
	void InitLUTStateActions(std::vector<stateRewardMap> * stateReward, std::vector<DataForObjects> & dataForObjects);

	/// return state_id reward pair for a given state
	actionReward Find(intVec & state);
	/// return state_id reward pair for a given state_id
	actionReward Find(int state_id);

	/// return state_id given state and gridSize
	static int State2Idx(intVec & state, int gridSize);
	/// return state_id given 2 vectors of moving objects, shelters and gridSize
	static int State2Idx(intVec & movingObj, intVec & shelters, int gridSize);

	static intVec Idx2State(int state_id, int stateSize, int gridSize);

	/// write map to file
	friend std::ofstream& operator<<(std::ofstream& out, const StateActionLUT& map);
	/// read map from file
	friend std::ifstream& operator>>(std::ifstream& in, StateActionLUT& map);

	int GetSarsopGridSize(int idxSarsop) const { return m_objectsNum[idxSarsop][GRIDSIZE]; };

private:
	int m_despotGridSize;
	/// numbers of objects (self, enemies, non-involved, shelters)
	std::vector<intVec> m_objectsNum;
	/// lut connecting state_id to preferable action and expected reward(normalized)
	stateActionMap m_stateActionMap;
	/// normalized sarsop lut lut[state] = reward
	std::vector<stateRewardMap> * m_stateRewardMap;
	/// self object of sarsop data
	std::vector<Self_Obj> m_self;
	/// enemy object for sarsop data
	std::vector<Attack_Obj> m_enemy;

	enum LUT_TYPE m_lutType;

	/// recursive function for init look up table (update while running parameters)
	void InitLutRec(intVec & beliefState, intVec & identityVec, int currObj, int backupSize);

	/// make descision by scaling the observed world to smaller scale
	actionReward BasicDecision(const intVec & beliefState, const intVec & identityVec) const;

	/// weight of all lut decision
	actionReward WeightedLUTDecision(const intVec & beliefState, const intVec & identityVec, enum LUT_TYPE weightType) const;

	/// add to current reward to sumReward vec and update count of rewards for each action
	void AddRewards(std::vector <double> & rewards, std::vector <double> & sumRewards, intVec & countLegal) const;
	
	/// insert maximum reward to the corresponding place in maxRewards
	void MaxRewards(std::vector <double> & rewards, std::vector <double> & maxRewards) const;
	
	/// insert minimum reward to the corresponding place in maxRewards
	void MinRewards(std::vector <double> & rewards, std::vector <double> & maxRewards) const;

	/// divide sumRewards by corresponding count for average
	void DivideByCount(std::vector <double> & sumRewards, intVec & countLegal) const;

	/// return the prefered action to local optimization
	void CreateRewardMatBasic(const intVec & beliefState, intVec & scaledState, const intVec & identityVec, int keyIdx, std::vector <double> & expectedRewards) const;

	/// return the prefered action to local optimization
	void CreateRewardMatJustEnemy(const intVec & beliefState, intVec & scaledState, const intVec & identityVec, int keyIdx, std::vector <double> & expectedRewards) const;

	/// return max reward action
	actionReward FindMaxRewardAction(std::vector <double> & expectedRewards) const;

	void DropNonExistObjects(intVec & localBeliefState, intVec & scaledState, intVec & identityVec, const intVec & key) const;

	/// return identity of the object according to max key
	enum OBJECT WhoAmI(int objIdx) const;

	/// return true if the object idx location does not repeat in state
	static bool NoRepetitions(intVec & state, int currIdx, int gridSize);
	/// return true if the state is with no repetitions for all idx until endCheck
	static bool NoRepetitionsAll(intVec & state, int gridSize, int endCheck);

	/// return true of coordinate is in frame
	static bool CoordInFrame(const Coordinate & point, const Coordinate & leftUpperCorner, int frameSize);

	/// update reward for move to target action
	void UpdateRewardMoveToTarget(intVec & scaledState, const intVec & identityVec, int keyIdx, double & reward) const;
	void MakeMoveToTarget(intVec & scaledState, const intVec & identityVec, int sarsopGridSize) const;

	/// update reward for move to target action
	void UpdateRewardMoveToShelter(const intVec & beliefState, intVec & scaledState, const intVec & identityVec, int keyIdx, double & reward) const;
	void MakeMoveToShelter(intVec & scaledState, const intVec & identityVec, int sarsopGridSize) const;

	/// update reward for move from enemy action
	void UpdateRewardMoveFromEnemy(intVec & scaledState, const intVec & identityVec, int keyIdx, double & reward) const;
	void MakeMoveFromEnemy(intVec & scaledState, int sarsopGridSize) const;

	/// update reward for move to target action
	void UpdateRewardAttack(intVec & scaledState, const intVec & identityVec, int keyIdx, double & reward) const;

	/// move to location, update possible move outcomes
	int MoveToLocation(intVec & state, Coordinate & goTo, int sarsopGridSize) const;

	/// move enemy location to the closest scaled spot but current scaled location
	void MoveEnemyLocation(const intVec & beliefState, intVec & scaledState, int numMovingObj, int keyIdx) const;

	/// in case self is on target after scaling we need to shift it from target
	void ShiftSelfFromTarget(const intVec & beliefState, intVec & scaledState, const intVec & identityVec, int sarsopGridSize) const;

	/// shift state left on grid
	void ShiftGridLeft(intVec & scaledState, const intVec & identityVec, int sarsopGridSize) const;
	/// shift state left on grid
	void ShiftGridUp(intVec & scaledState,const intVec & identityVec, int sarsopGridSize) const;

	/// drop shelter from state
	void DropShelter(intVec & state, const intVec & identityVec, int sarsopGridSize) const;
	
	/// drop non protected shelters from state
	void DropNonProtectedShelter(const intVec & beliefState, intVec & scaledState, const intVec & identityVec, int sarsopGridSize) const;

	/// return the idx of the coord given gridSize
	int CoordToIdx(const Coordinate & location, int gridSize) const;

	/// init vector of only moving object locations
	void InitMovingObjVec(intVec & state, intVec & movingObj, const intVec & identityVec) const;
	
	/// return true if location is valid for a given state (no repeats)
	static bool ValidLocation(intVec & state, int location);

	/// return true if location is valid for a given state (no repeats and inside grid)
	static bool ValidLegalLocation(intVec & state, const Coordinate & location, int end, int gridSize);

	///return true if there is a dead enemy
	bool EnemyDead(intVec & scaledState, const intVec & identityVec, int gridSize) const;
	///return true if there is a dead non involved
	bool NonInvDeadByAttack(intVec & afterAttack, intVec & beforeAttack, const intVec & identityVec, int gridSize) const;
};

