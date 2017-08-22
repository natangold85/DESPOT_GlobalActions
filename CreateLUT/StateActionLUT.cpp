#include "StateActionLUT.h"

static const char * BACKUPFILENAME = "backup.bin";
std::ofstream backupLUTWrite;
std::ifstream backupLUTRead;
int counter[4] = { 0,0,0,0 };

inline int Max(int a, int b)
{
	return a * (a >= b) + b * (a < b);
}

inline double Max(double a, double b)
{
	return a * (a >= b) + b * (a < b);
}

inline double Min(double a, double b)
{
	return b * (a >= b) + a * (a < b);
}

/// return absolute value of a number
inline int Abs(int num)
{
	return num * (num >= 0) - num * (num < 0);
}

inline int Distance(Coordinate & a, Coordinate &  b)
{
	return (a.X() - b.X()) * (a.X() - b.X()) + (a.Y() - b.Y()) * (a.Y() - b.Y());
}

inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;
	return xDiff * xDiff + yDiff * yDiff;
}

inline double RealDistance(Coordinate & a, Coordinate &  b)
{
	return sqrt((a.X() - b.X()) * (a.X() - b.X()) + (a.Y() - b.Y()) * (a.Y() - b.Y()));
}

std::vector<int> MaxKey(std::vector<std::vector<int>> & objectsNum)
{
	std::vector<int> maxKey(objectsNum[0].size());

	for (size_t i = 0; i < objectsNum[0].size(); ++i)
	{
		// find max of each slot in vector
		int max = 0;
		for (size_t j = 0; j < objectsNum.size(); ++j)
		{
			max = Max(objectsNum[j][i], max);
		}
		maxKey[i] = max;
	}

	return maxKey;
}
/// changes of possible moves from a point ((0,0) must be the last entry for this moves lut)
static const int s_numPossibleMoves = 8;
static const int s_lutMoves[s_numPossibleMoves][2] = { { 0,1 },{ 0,-1 },{ 1,0 },{ -1,0 },{ 1,1 },{ 1,-1 },{ -1,1 },{ -1,-1 } }; // 

StateActionLUT::StateActionLUT(int despotGridSize)
	: m_despotGridSize(despotGridSize)
	, m_objectsNum()
	, m_stateActionMap()
	, m_stateRewardMap()
	, m_self()
	, m_enemy()
	, m_lutType()
{}

StateActionLUT::StateActionLUT(std::vector<intVec> & keys, int onlineGridSize, LUT_TYPE type)
: m_despotGridSize(onlineGridSize)
, m_objectsNum(keys)
, m_stateActionMap()
, m_stateRewardMap()
, m_self()
, m_enemy()
, m_lutType(type)
{
	if (keys.size() > 1)
	{
		// insert max key as the last key
		m_objectsNum.emplace_back(MaxKey(keys));
	}
}

void StateActionLUT::InitLUTStateActions(std::vector<stateRewardMap> * stateReward, std::vector<DataForObjects> & dataForObjects)
{
	for (auto data : dataForObjects)
	{
		m_self.emplace_back(Self_Obj(Point(), Move_Properties(0.0, data.m_moveProb), data.m_attackRange, data.m_pHit, data.m_observationRange, data.m_pObservation));
		m_enemy.emplace_back(Attack_Obj(Point(), Move_Properties(), data.m_attackRangeEnemy, data.m_pHit));
	}
	
	int maxIdx = m_objectsNum.size() - 1;
	int MaxNumObjects = 1 + m_objectsNum[maxIdx][1] + m_objectsNum[maxIdx][2] + m_objectsNum[maxIdx][3];

	intVec beliefState(MaxNumObjects);
	intVec identityVec(MaxNumObjects);
	m_stateRewardMap = stateReward;
	InitLutRec(beliefState, identityVec, 0, 0);

	std::cout << "counter:\n";
	for (int i = 0; i < 4; ++i)
		std::cout << "action " << i << " = " << counter[i] << "\n";
}

StateActionLUT::actionReward StateActionLUT::Find(intVec & state)
{
	return Find(State2Idx(state, m_despotGridSize));
}

StateActionLUT::actionReward StateActionLUT::Find(int state_id)
{
	auto itr = m_stateActionMap.find(state_id);

	if (itr == m_stateActionMap.end())
		return std::make_pair(NUM_ACTIONS, -100);

	return itr->second;
}

int StateActionLUT::State2Idx(intVec & state, int gridSize)
{
	int idx = 0;
	// add 1 for num states for dead objects
	int numStates = gridSize * gridSize + 1;
	// idx = s[0] * numstates^n + s[1] * numstates^(n - 1) ...  + s[n] * numstates^(0)
	for (int i = 0; i < state.size(); ++i)
	{
		idx *= numStates;
		idx += state[i];
	}

	return idx;
}

StateActionLUT::intVec StateActionLUT::Idx2State(int state_id, int stateSize, int gridSize)
{
	intVec state(stateSize);
	int numStates = gridSize * gridSize + 1;

	// running on all varied objects and concluding from the obs num the observed state
	for (int i = stateSize - 1; i >= 0; --i)
	{
		state[i] = state_id % numStates;
		state_id /= numStates;
	}

	return state;
}

int StateActionLUT::State2Idx(intVec & movingObj, intVec & shelters, int gridSize)
{
	int idx = 0;
	// add 1 for num states for dead objects
	int numStates = gridSize * gridSize + 1;
	// idx = s[0] * numstates^n + s[1] * numstates^(n - 1) ...  + s[n] * numstates^(0)
	for (int i = 0; i < movingObj.size(); ++i)
	{
		idx *= numStates;
		idx += movingObj[i];
	}

	for (int i = 0; i < shelters.size(); ++i)
	{
		idx *= numStates;
		idx += shelters[i];
	}

	return idx;
}

void StateActionLUT::InitLutRec(intVec & beliefState, intVec & identityVec, int currObj, int backupSize)
{
	if (currObj == beliefState.size())
	{
		actionReward action;
		if (m_lutType == SINGLE)
			action = BasicDecision(beliefState, identityVec);
		else 
			action = WeightedLUTDecision(beliefState, identityVec, m_lutType);
		
		++counter[action.first];
		if (action.first != NUM_ACTIONS)
		{
			m_stateActionMap[State2Idx(beliefState, m_despotGridSize)] = action;
		}
	}
	else
	{
		int numStates = m_despotGridSize * m_despotGridSize;
		identityVec[currObj] = WhoAmI(currObj);

		for (size_t i = 0; i < numStates; ++i)
		{
			// in case of self recursion writing and reading backup if necessary
			if (identityVec[currObj] == SELF)
			{
				beliefState[currObj] = i;
				InitLutRec(beliefState, identityVec, currObj + 1, backupSize);
				std::cout << i << ", ";
			}
			else
			{
				beliefState[currObj] = i;
				if (identityVec[currObj] == SHELTER || NoRepetitions(beliefState, currObj, m_despotGridSize))
					InitLutRec(beliefState, identityVec, currObj + 1, backupSize);
			}
		}

		if (identityVec[currObj] == ENEMY)
		{
			beliefState[currObj] = m_despotGridSize * m_despotGridSize;
			InitLutRec(beliefState, identityVec, currObj + 1, backupSize);
		}
	}
}

StateActionLUT::actionReward StateActionLUT::BasicDecision(const intVec & beliefState, const intVec & identityVec) const
{
	// if enemy is dead automaticly return move to target
	if (beliefState[1] == m_despotGridSize * m_despotGridSize)
		return actionReward(MOVE_TO_TARGET, 0.5);

	int sarsopGridSize = m_objectsNum[0][GRIDSIZE];
	double scale = static_cast<double>(m_despotGridSize) / sarsopGridSize;
	intVec scaledState(beliefState.size());

	for (size_t i = 0; i < beliefState.size(); ++i)
	{
		Coordinate location(beliefState[i] % m_despotGridSize, beliefState[i] / m_despotGridSize);
		location /= scale;		
		scaledState[i] = location.X() + location.Y() * sarsopGridSize;
	}

	// if the enemy is in the same spot move enemy to the neares available location
	if (!NoRepetitions(scaledState, 1, sarsopGridSize))
		MoveEnemyLocation(beliefState, scaledState, 1 + m_objectsNum[0][1] + m_objectsNum[0][2], sarsopGridSize);

	// if target is in self location shift map to the left or upper so self won't be in target location 
	if (scaledState[0] == sarsopGridSize * sarsopGridSize - 1)
		ShiftSelfFromTarget(beliefState, scaledState, identityVec, sarsopGridSize);

	// if the non-involved is in non-valid location don't count him
	if (m_objectsNum[0][NUM_NON_INVOLVED] == 1 && !NoRepetitions(scaledState, m_objectsNum[0][NUM_ENEMIES] + 1, sarsopGridSize))
	{
		scaledState[m_objectsNum[0][NUM_ENEMIES] + 1] = sarsopGridSize * sarsopGridSize;
	}

	std::vector<double> expectedRewards(NUM_ACTIONS);
	for (auto v : expectedRewards)
		v = -1000.0;

	if (beliefState.size() > 2)
		CreateRewardMatBasic(beliefState, scaledState, identityVec, 0, expectedRewards);
	else if (beliefState.size() == 2)
		CreateRewardMatJustEnemy(beliefState, scaledState, identityVec, 0, expectedRewards);
	
	return FindMaxRewardAction(expectedRewards);
}

StateActionLUT::actionReward  StateActionLUT::WeightedLUTDecision(const intVec & beliefState, const intVec & identityVec, enum LUT_TYPE weightType) const
{
	double initRewardVal = weightType == MAX ? -1000.0 : 0.0;
	// if enemy is dead automaticly return move to target
	if (beliefState[1] == m_despotGridSize * m_despotGridSize)
		return actionReward(MOVE_TO_TARGET, 0.5);

	std::vector<double> weightedRewards(NUM_ACTIONS, initRewardVal);

	std::vector<int> countLegal(NUM_ACTIONS, 0);

	// run on all keys but max key and calculate reward matrix for them
	for (size_t i = 0; i < m_objectsNum.size() - 1; ++i)
	{
		int sarsopGridSize = m_objectsNum[i][GRIDSIZE];
		double scale = static_cast<double>(m_despotGridSize) / sarsopGridSize;
		intVec scaledState(beliefState.size());

		for (size_t i = 0; i < beliefState.size(); ++i)
		{
			Coordinate location(beliefState[i] % m_despotGridSize, beliefState[i] / m_despotGridSize);
			location /= scale;

			scaledState[i] = location.X() + location.Y() * sarsopGridSize;
		}

		// adjust state to local key
		intVec localIdentity(identityVec);
		intVec localBeliefState(beliefState);
		DropNonExistObjects(localBeliefState, scaledState, localIdentity, m_objectsNum[i]);

		// if the enemy is in the same spot move enemy to the nearest available location
		if (!NoRepetitions(scaledState, 1, sarsopGridSize))
			MoveEnemyLocation(localBeliefState, scaledState, 1 + m_objectsNum[i][1] + m_objectsNum[i][2], i);

		// if target is in self location shift map to the left or upper so self won't be in target location 
		if (scaledState[0] == sarsopGridSize * sarsopGridSize - 1)
			ShiftSelfFromTarget(localBeliefState, scaledState, localIdentity, sarsopGridSize);

		// if the non-involved is in non-valid location don't count him
		if (m_objectsNum[i][NUM_NON_INVOLVED] == 1 && !NoRepetitions(scaledState, m_objectsNum[i][NUM_ENEMIES] + 1, sarsopGridSize))
		{
			scaledState[m_objectsNum[i][NUM_ENEMIES] + 1] = sarsopGridSize * sarsopGridSize;
		}

		std::vector<double> expectedRewards(NUM_ACTIONS, -1000.0);

		if (localBeliefState.size() > 2)
			CreateRewardMatBasic(localBeliefState, scaledState, localIdentity, i, expectedRewards);
		else if (localBeliefState.size() == 2)
			CreateRewardMatJustEnemy(localBeliefState, scaledState, localIdentity, i, expectedRewards);

		if (weightType == AVG)
			AddRewards(expectedRewards, weightedRewards, countLegal);
		else if (weightType == MAX)
			MaxRewards(expectedRewards, weightedRewards);
		else if (weightType == MIN)
			MinRewards(expectedRewards, weightedRewards);
	}
	if (weightType == AVG)
		DivideByCount(weightedRewards, countLegal);

	return FindMaxRewardAction(weightedRewards);
}

void StateActionLUT::AddRewards(std::vector<double>& rewards, std::vector<double>& sumRewards, intVec & countLegal) const
{
	for (size_t i = 0; i < rewards.size(); ++i)
	{
		if (rewards[i] != -1000.0)
		{
			sumRewards[i] += rewards[i];
			++countLegal[i];
		}
	}
}

void StateActionLUT::MaxRewards(std::vector<double>& rewards, std::vector<double>& maxRewards) const
{
	for (size_t i = 0; i < rewards.size(); ++i)
	{
		maxRewards[i] = Max(rewards[i], maxRewards[i]);
	}
}

void StateActionLUT::MinRewards(std::vector<double>& rewards, std::vector<double>& maxRewards) const
{
	for (size_t i = 0; i < rewards.size(); ++i)
	{
		maxRewards[i] = Min(rewards[i], maxRewards[i]);
	}
}

void StateActionLUT::DivideByCount(std::vector<double>& sumRewards, intVec & countLegal) const
{
	for (size_t i = 0; i < sumRewards.size(); ++i)
	{
		if (countLegal[i] != 0)
			sumRewards[i] /= countLegal[i];
		else
			sumRewards[i] = -1000.0;
	}
}

enum StateActionLUT::OBJECT StateActionLUT::WhoAmI(int objIdx) const
{
	int maxKeyIdx = m_objectsNum.size() - 1;
	return objIdx == 0 ? SELF :
		objIdx < 1 + m_objectsNum[maxKeyIdx][NUM_ENEMIES] ? ENEMY :
		objIdx < 1 + m_objectsNum[maxKeyIdx][NUM_ENEMIES] + m_objectsNum[maxKeyIdx][NUM_NON_INVOLVED] ? NON_INVOLVED : SHELTER;
}

bool StateActionLUT::NoRepetitions(intVec & state, int currIdx, int gridSize)
{
	for (int i = 0; i < currIdx; ++i)
	{
		if (state[i] == state[currIdx] && state[i] != gridSize * gridSize)
		{
			return false;
		}
	}
	return true;
}

bool StateActionLUT::NoRepetitionsAll(intVec & state, int gridSize, int endCheck)
{
	bool status = true;
	for (int i = 1; i < endCheck && status; ++i)
		status = NoRepetitions(state, i, gridSize);

	return status;
}

bool StateActionLUT::CoordInFrame(const Coordinate & point, const Coordinate & leftUpperCorner, int frameSize)
{
	return (point.X() >= leftUpperCorner.X()) & (point.X() < leftUpperCorner.X() + frameSize) &&
		(point.Y() >= leftUpperCorner.Y()) & (point.Y() < leftUpperCorner.Y() + frameSize);
}

void StateActionLUT::CreateRewardMatBasic(const intVec & beliefState, intVec & scaledState, const intVec& identityVec, int keyIdx, std::vector <double> & expectedRewards) const
{
	int sarsopGridSize = m_objectsNum[keyIdx][GRIDSIZE];

	// update reward move to shelter
	UpdateRewardMoveToShelter(beliefState, scaledState, identityVec, keyIdx, expectedRewards[MOVE_TO_SHELTER]);
	
	// drop non-protected shelters for scaling reasons
	DropNonProtectedShelter(beliefState, scaledState, identityVec, sarsopGridSize);

	// update reward for other moves
	UpdateRewardMoveToTarget(scaledState, identityVec, keyIdx, expectedRewards[MOVE_TO_TARGET]);
	UpdateRewardMoveFromEnemy(scaledState, identityVec, keyIdx, expectedRewards[MOVE_FROM_ENEMY]);
	UpdateRewardAttack(scaledState, identityVec, keyIdx, expectedRewards[ATTACK]);
}

void StateActionLUT::CreateRewardMatJustEnemy(const intVec & beliefState, intVec & scaledState, const intVec & identityVec, int keyIdx, std::vector <double> & expectedRewards) const
{
	// shelter does not exist the move is not legal
	expectedRewards[MOVE_TO_SHELTER] = -1000.0;

	// update reward for other moves
	UpdateRewardMoveToTarget(scaledState, identityVec, keyIdx, expectedRewards[MOVE_TO_TARGET]);
	UpdateRewardMoveFromEnemy(scaledState, identityVec, keyIdx, expectedRewards[MOVE_FROM_ENEMY]);
	UpdateRewardAttack(scaledState, identityVec, keyIdx, expectedRewards[ATTACK]);
}

StateActionLUT::actionReward StateActionLUT::FindMaxRewardAction(std::vector<double>& expectedRewards) const
{
	actionReward maxReward(-1, -1000);
	for (size_t i = 0; i < expectedRewards.size(); ++i)
	{
		if (maxReward.second < expectedRewards[i])
		{
			maxReward.first = i;
			maxReward.second = expectedRewards[i];
		}
	}

	return maxReward;
}

void StateActionLUT::DropNonExistObjects(intVec & localBeliefState, intVec & scaledState, intVec & identityVec, const intVec & key) const
{
	int maxKeyIdx = m_objectsNum.size() - 1;
	if (key[NUM_ENEMIES] != m_objectsNum[maxKeyIdx][NUM_ENEMIES])
	{
		localBeliefState.erase(localBeliefState.begin() + 1);
		scaledState.erase(scaledState.begin() + 1);
		identityVec.erase(identityVec.begin() + 1);
	}

	if (key[NUM_NON_INVOLVED] != m_objectsNum[maxKeyIdx][NUM_NON_INVOLVED])
	{
		localBeliefState.erase(localBeliefState.begin() + 1 + key[NUM_ENEMIES]);
		scaledState.erase(scaledState.begin() + 1 + key[NUM_ENEMIES]);
		identityVec.erase(identityVec.begin() + 1 + key[NUM_ENEMIES]);
	}

	if (key[NUM_SHELTERS] != m_objectsNum[maxKeyIdx][NUM_SHELTERS])
	{
		localBeliefState.erase(localBeliefState.begin() + 1 + key[NUM_ENEMIES] + key[NUM_NON_INVOLVED]);
		scaledState.erase(scaledState.begin() + 1 + key[NUM_ENEMIES] + key[NUM_NON_INVOLVED]);
		identityVec.erase(identityVec.begin() + 1 + key[NUM_ENEMIES] + key[NUM_NON_INVOLVED]);
	}
}

void StateActionLUT::UpdateRewardMoveToTarget(intVec & scaledState, const intVec& identityVec, int keyIdx, double & reward) const
{
	// add reward if move failed
	double rewardFailMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, m_objectsNum[keyIdx][GRIDSIZE])];
	reward = (1 - m_self[keyIdx].GetMovement().GetToward()) * rewardFailMove;
	
	// add reward if move succeed
	int remember = scaledState[0];
	MakeMoveToTarget(scaledState, identityVec, m_objectsNum[keyIdx][GRIDSIZE]);
	double rewardMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, m_objectsNum[keyIdx][GRIDSIZE])];
	reward += m_self[keyIdx].GetMovement().GetToward() * rewardMove;
	scaledState[0] = remember;
}

void StateActionLUT::MakeMoveToTarget(intVec & scaledState, const intVec& identityVec, int sarsopGridSize) const
{
	// create state complies only with moving objects(blocking objects)
	intVec movingObjState;
	InitMovingObjVec(scaledState, movingObjState, identityVec);
	Coordinate target(sarsopGridSize - 1, sarsopGridSize - 1);
	
	int newLoc = MoveToLocation(movingObjState, target, sarsopGridSize);
	// if move to target is legit make it
	if (newLoc != -1)
		scaledState[0] = newLoc;
}

void StateActionLUT::UpdateRewardMoveToShelter(const intVec & beliefState, intVec & scaledState, const intVec& identityVec, int keyIdx, double & reward) const
{
	int sarsopGridSize = m_objectsNum[keyIdx][GRIDSIZE];
	
	// if we already in shelter or the shelter is not in range the reward = reward of current state including shelter
	if (beliefState[0] == beliefState[beliefState.size() - 1] | scaledState[scaledState.size() - 1] == sarsopGridSize * sarsopGridSize)
	{
		reward = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
	}
	else if (scaledState[0] == scaledState[scaledState.size() - 1]) // if shelter is around us calculate as move to shelter will make us in shelter TODO : need to adjust to number of steps to shelter
	{
		// calculate reward if move failed
		intVec stateWOShelter(scaledState);
		// drop shelters
		DropShelter(stateWOShelter, identityVec, sarsopGridSize);
		double rewardFailMove = (*m_stateRewardMap)[keyIdx][State2Idx(stateWOShelter, sarsopGridSize)];
		reward = (1 - m_self[keyIdx].GetMovement().GetToward()) * rewardFailMove;

		// calculate reward if move succeed
		double rewardSafe = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
		reward += m_self[keyIdx].GetMovement().GetToward() * rewardSafe;
	}
	else // else calculate regular move to shelter
	{
		// calculate reward if move failed
		double rewardFailMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
		reward = (1 - m_self[keyIdx].GetMovement().GetToward()) * rewardFailMove;
		
		// calculate reward if move succeed
		int remember = scaledState[0];
		MakeMoveToShelter(scaledState, identityVec, sarsopGridSize);
		double rewardMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
		reward += m_self[keyIdx].GetMovement().GetToward() * rewardMove;
		scaledState[0] = remember;
	}
}

void StateActionLUT::MakeMoveToShelter(intVec & scaledState, const intVec& identityVec, int sarsopGridSize) const
{
	intVec movingObjState;
	InitMovingObjVec(scaledState, movingObjState, identityVec);

	Coordinate shelter;
	// TODO: is not support in more than 1 shelter
	for (size_t i = 0; i < scaledState.size(); ++i)
		if (identityVec[i] == SHELTER)
		{
			shelter.X() = scaledState[i] % sarsopGridSize;
			shelter.Y() = scaledState[i] / sarsopGridSize;
		}

	// find colsest point to shelter location shelter location
	int newLoc = MoveToLocation(movingObjState, shelter, sarsopGridSize);
	// if location exist change state
	if (newLoc != -1)
		scaledState[0] = newLoc;
}

void StateActionLUT::UpdateRewardMoveFromEnemy(intVec & scaledState, const intVec & identityVec, int keyIdx, double & reward) const
{
	int sarsopGridSize = m_objectsNum[keyIdx][GRIDSIZE];
	// create vector of only moving objects(blocking objects)
	intVec movingObjState;
	InitMovingObjVec(scaledState, movingObjState, identityVec);

	// make move from enemy
	MakeMoveFromEnemy(movingObjState, sarsopGridSize);

	// calculate reward if move failed
	double rewardFailMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
	reward = (1 - m_self[keyIdx].GetMovement().GetToward()) * rewardFailMove;

	// calculate reward if move succeed
	int remember = scaledState[0];
	scaledState[0] = movingObjState[0];
	double rewardMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
	reward += m_self[keyIdx].GetMovement().GetToward() * rewardMove;
	scaledState[0] = remember;
}

void StateActionLUT::MakeMoveFromEnemy(intVec & scaledState, int sarsopGridSize) const
{
	Coordinate self(scaledState[0] % sarsopGridSize, scaledState[0] / sarsopGridSize);
	Coordinate enemy(scaledState[1] % sarsopGridSize, scaledState[1] / sarsopGridSize);

	int maxDist = Distance(self, enemy);
	int idxOfMax = -1;
	// run on all possible moves and find the farthest point from enemy
	for (size_t i = 0; i < s_numPossibleMoves; ++i)
	{
		// change self according to move
		Coordinate newLoc (self.X() + s_lutMoves[i][0], self.Y() + s_lutMoves[i][1]);

		if (ValidLegalLocation(scaledState, newLoc, scaledState.size(), sarsopGridSize))
		{
			int dist = Distance(newLoc, enemy);
			if (dist > maxDist)
			{
				maxDist = dist;
				idxOfMax = i;
			}
		}
	}
	// if there is farther location from enemy change self location
	if (idxOfMax != -1)
	{
		self.X() += s_lutMoves[idxOfMax][0];
		self.Y() += s_lutMoves[idxOfMax][1];
		scaledState[0] = self.X() + self.Y() * sarsopGridSize;
	}
}

void StateActionLUT::UpdateRewardAttack(intVec & scaledState, const intVec& identityVec, int keyIdx, double & reward) const
{
	// create vectors for attack and move calculations
	intVec movingObjState;
	intVec shelters;
	for (size_t i = 0; i < scaledState.size(); ++i)
	{
		if (identityVec[i] < SHELTER)
			movingObjState.emplace_back(scaledState[i]);
		else
			shelters.emplace_back(scaledState[i]);
	}

	int sarsopGridSize = m_objectsNum[keyIdx][GRIDSIZE];
	Coordinate self(scaledState[0] % sarsopGridSize, scaledState[0] / sarsopGridSize);
	Coordinate enemy(scaledState[1] % sarsopGridSize, scaledState[1] / sarsopGridSize);

	double dist = RealDistance(self, enemy);

	// if enemy is in range attack o.w. move to enemy
	if (dist <= m_self[keyIdx].GetRange())
	{
		std::vector<std::pair<intVec, double>> attackResult;
		m_self[keyIdx].CalcSelfAttackSARSOP(movingObjState, shelters, sarsopGridSize, attackResult);
		
		reward = 0.0;
		// run on attack result and add its relative reward to reward
		for (auto v : attackResult)
		{
			// normalization of the meaning of a dead enemy (TODO : find better solution for it)
			if (EnemyDead(v.first, identityVec, sarsopGridSize))
				reward += 0.35 * v.second;
			else if (NonInvDeadByAttack(v.first, movingObjState, identityVec, sarsopGridSize))
				reward += -1.0 * v.second;
			else
			{
				reward += (*m_stateRewardMap)[keyIdx][State2Idx(v.first, shelters, sarsopGridSize)] * v.second;
			}
		}
	}
	else
	{
		// calclate reward if move failed
		double rewardFailMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
		reward = (1 - m_self[keyIdx].GetMovement().GetToward()) * rewardFailMove;

		// calculate reward if move succeed
		int remember = scaledState[0];
		int loc = MoveToLocation(movingObjState, enemy, sarsopGridSize);
		if (loc != -1)
			scaledState[0] = loc;
		
		double rewardMove = (*m_stateRewardMap)[keyIdx][State2Idx(scaledState, sarsopGridSize)];
		reward += m_self[keyIdx].GetMovement().GetToward() * rewardMove;
		scaledState[0] = remember;
	}
}

int StateActionLUT::MoveToLocation(intVec & state, Coordinate & goTo, int sarsopGridSize) const
{
	int selfLocation = state[0];
	int goToLoc = goTo.X() + goTo.Y() * sarsopGridSize;

	int xDiff = goTo.X() - selfLocation % sarsopGridSize;
	int yDiff = goTo.Y() - selfLocation / sarsopGridSize;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * sarsopGridSize : 0;

	int move = selfLocation + changeToInsertX + changeToInsertY;

	// if the best move is valid return it else if there is only one direction to advance return -1
	if (ValidLocation(state, move))
		return move;
	else if (changeToInsertX == 0 | changeToInsertY == 0)
		return -1;

	// try move to in the axis in which we are farther than goTo
	int secondMove;
	if (Distance(goToLoc, selfLocation + changeToInsertX, sarsopGridSize) > Distance(goToLoc, selfLocation + changeToInsertY, sarsopGridSize))
	{
		move = selfLocation + changeToInsertX;
		secondMove = selfLocation + changeToInsertY;
	}
	else
	{
		secondMove = selfLocation + changeToInsertX;
		move = selfLocation + changeToInsertY;
	}

	if (ValidLocation(state, move))
		return move;

	if (ValidLocation(state, secondMove))
		return secondMove;

	return -1;
}

void StateActionLUT::MoveEnemyLocation(const intVec & beliefState, intVec & scaledState, int endMovingObj, int keyIdx) const
{
	int sarsopGridSize = m_objectsNum[keyIdx][GRIDSIZE];
	Coordinate self(scaledState[0] % sarsopGridSize, scaledState[0] / sarsopGridSize);
	Coordinate realEnemy(beliefState[1] % m_despotGridSize, beliefState[1] / m_despotGridSize);
	int minDist = 100000;
	int bestLocation = -1;
	double scaleBackward = static_cast<double>(sarsopGridSize) / m_despotGridSize;

	// run on all possible moves of enemy and calculate which one is closest to enemy
	for (size_t i = 0; i < s_numPossibleMoves; ++i)
	{
		Coordinate newLoc(s_lutMoves[i][0], s_lutMoves[i][1]);
		newLoc += self;
		if (ValidLegalLocation(scaledState, newLoc, endMovingObj, sarsopGridSize))
		{
			newLoc /= scaleBackward;
			int dist = Distance(newLoc, realEnemy);
			if (dist < minDist)
			{
				minDist = dist;
				bestLocation = i;
			}
		}
	}
	
	scaledState[1] += s_lutMoves[bestLocation][0] + s_lutMoves[bestLocation][1] * sarsopGridSize;
	
	// assumption : only 1 shelter
	// if the enemy is closer to shelter move it to the new enemy location
	int shelterIdx = 1 + m_objectsNum[keyIdx][ENEMY] + m_objectsNum[keyIdx][NON_INVOLVED];
	if (m_objectsNum[keyIdx][SHELTER] == 0 || beliefState[shelterIdx] == m_despotGridSize * m_despotGridSize)
		return;

	int distSelf = Distance(beliefState[0], beliefState[shelterIdx], m_despotGridSize);
	int distEnemy = Distance(beliefState[1], beliefState[shelterIdx], m_despotGridSize);
	if (distEnemy < distSelf)
	{
		scaledState[shelterIdx] = scaledState[1];
	}
}

void StateActionLUT::ShiftSelfFromTarget(const intVec & beliefState, intVec & scaledState, const intVec& identityVec, int sarsopGridSize) const
{
	Coordinate realSelf(beliefState[0] % m_despotGridSize, beliefState[0] / m_despotGridSize);
	if (realSelf.X() > realSelf.Y())
		ShiftGridLeft(scaledState, identityVec, sarsopGridSize);
	else
		ShiftGridUp(scaledState, identityVec, sarsopGridSize);
}

void StateActionLUT::InitMovingObjVec(intVec & state, intVec & movingObj, const intVec& identityVec) const
{
	for (size_t i = 0; i < state.size(); ++i)
	{
		if (identityVec[i] < SHELTER)
			movingObj.emplace_back(state[i]);
	}
}

void StateActionLUT::ShiftGridLeft(intVec & scaledState, const intVec& identityVec, int sarsopGridSize) const
{
	for (int i = 0; i < scaledState.size(); ++i)
	{
		Coordinate point(scaledState[i] % sarsopGridSize, scaledState[i] / sarsopGridSize);
		--point.X();

		if (point.X() < 0)
		{
			scaledState[i] = sarsopGridSize * sarsopGridSize;
		}
		else
		{
			--scaledState[i];
		}
	}
}

void StateActionLUT::ShiftGridUp(intVec & scaledState, const intVec& identityVec, int sarsopGridSize) const
{
	for (int i = 0; i < scaledState.size(); ++i)
	{
		Coordinate point(scaledState[i] % sarsopGridSize, scaledState[i] / sarsopGridSize);
		--point.Y();

		// if the object is not close to self drop him from calculation
		if (point.Y() < 0)
		{
			scaledState[i] = sarsopGridSize * sarsopGridSize;
		}
		else
		{
			scaledState[i] -= sarsopGridSize;
		}
	}
}

void StateActionLUT::DropShelter(intVec & state, const intVec & identityVec, int sarsopGridSize) const
{
	for (size_t i = 0; i < state.size(); ++i)
	{
		if (identityVec[i] == SHELTER)
		{
			state[i] = sarsopGridSize * sarsopGridSize;
		}
	}
}

void StateActionLUT::DropNonProtectedShelter(const intVec & beliefState, intVec & scaledState, const intVec& identityVec, int sarsopGridSize) const
{
	for (size_t i = 0; i < beliefState.size(); ++i)
	{
		if (identityVec[i] == SHELTER)
		{
			bool isProtected = false;
			for (size_t j = 0; identityVec[j] < SHELTER; ++j)
				isProtected |= beliefState[i] == beliefState[j];

			if (!isProtected)
			{
				scaledState[i] = sarsopGridSize * sarsopGridSize;
			}
		}
	}
}


inline int StateActionLUT::CoordToIdx(const Coordinate & location, int gridSize) const
{
	return location.X() + location.Y() * gridSize;
}


bool StateActionLUT::ValidLegalLocation(intVec & state, const Coordinate & location, int end, int gridSize)
{
	if ((location.X() >= gridSize) | (location.X() < 0) | (location.Y() >= gridSize) | (location.Y() < 0))
		return false;

	int locIdx = location.X() + location.Y() * gridSize;

	for (int i = 0; i < end; ++i)
	{
		if (locIdx == state[i])
		{
			return false;
		}
	}

	return true;
}

bool StateActionLUT::ValidLocation(intVec & state, int location)
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (location == state[i])
		{
			return false;
		}
	}

	return true;
}

bool StateActionLUT::EnemyDead(intVec & scaledState, const intVec & identityVec, int gridSize) const
{
	for (size_t i = 0; i < scaledState.size(); i++)
		if (scaledState[i] == gridSize * gridSize & identityVec[i] == ENEMY)
			return true;

	return false;
}

bool StateActionLUT::NonInvDeadByAttack(intVec & afterAttack, intVec & beforeAttack, const intVec & identityVec, int gridSize) const
{
	for (size_t i = 0; i < afterAttack.size(); i++)
		if (afterAttack[i] == gridSize * gridSize & identityVec[i] == NON_INVOLVED & beforeAttack[i] != gridSize * gridSize)
			return true;

	return false;
}

std::ofstream & operator<<(std::ofstream & out, const StateActionLUT & map)
{
	// write key of sarsop being used
	out.write(reinterpret_cast<const char *>(&map.m_objectsNum[0]), sizeof(int) * map.m_objectsNum.size());

	// write size of map
	int mapSize = map.m_stateActionMap.size();
	out.write(reinterpret_cast<char *>(&mapSize), sizeof(int));

	std::for_each(map.m_stateActionMap.begin(), map.m_stateActionMap.end(), [&out](StateActionLUT::stateActionMap::const_reference itr)
	{
		out.write(reinterpret_cast<const char *>(&itr.first), sizeof(int));
		// write action reward pair
		out.write(reinterpret_cast<const char *>(&itr.second.first), sizeof(int));
		out.write(reinterpret_cast<const char *>(&itr.second.second), sizeof(double));
	});

	return out;
}

std::ifstream & operator>>(std::ifstream & in, StateActionLUT & map)
{
	// read key
	map.m_objectsNum.resize(4);
	in.read(reinterpret_cast<char *>(&map.m_objectsNum[0]), sizeof(int) * map.m_objectsNum.size());

	// read map size
	int mapSize = 0;
	in.read(reinterpret_cast<char *>(&mapSize), sizeof(int));

	for (size_t i = 0; i < mapSize; ++i)
	{
		int state_id;
		in.read(reinterpret_cast<char *>(&state_id), sizeof(int));
		int action;
		in.read(reinterpret_cast<char *>(&action), sizeof(int));
		double reward;
		in.read(reinterpret_cast<char *>(&reward), sizeof(double));

		map.m_stateActionMap[state_id] = std::make_pair(action, reward);
	}

	return in;
}
