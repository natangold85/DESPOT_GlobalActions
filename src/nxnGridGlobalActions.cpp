#include <string>
#include <math.h>
#include <limits.h>

#include "..\include\despot\util\random.h"
#include "..\include\despot\solver\pomcp.h"

#include "nxnGridGlobalActions.h"
#include "Coordinate.h"

namespace despot 
{
/// changes surrounding a point
static const int s_numMoves = 8;
static const int s_lutDirections[s_numMoves][2] = { { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

static bool s_IS_MOVE_FROM_ENEMY = true;
/// string array for all actions (enum as idx)
static std::vector<std::string> s_ACTION_STR;

/// observation for loss and win (does not important)
static int OB_LOSS = 0;
static int OB_WIN = 0;

/* =============================================================================
* inline Functions
* =============================================================================*/

/// return absolute value of a number
inline int Abs(int num)
{
	return num * (num >= 0) - num * (num < 0);
}

/// return min(a,b)
inline int Min(int a, int b)
{
	return a * (a <= b) + b * (b < a);
}

/// return min(a,b)
inline int Max(int a, int b)
{
	return a * (a >= b) + b * (b > a);
}

/// return min(a,b)
inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;
	return xDiff * xDiff + yDiff * yDiff;
}


/* =============================================================================
* nxnGridGlobalActions Functions
* =============================================================================*/

nxnGridGlobalActions::nxnGridGlobalActions(int gridSize, int target, Self_Obj & self, std::vector<intVec> & objectsInitLoc, bool isMoveFromEnemyExist)
: nxnGrid(gridSize, target, self, objectsInitLoc)
{
	// init vector for string of actions
	s_ACTION_STR.clear();
	s_ACTION_STR.emplace_back("Move To Target");
	s_numBasicActions = NumBasicActions();
	s_numEnemyRelatedActions = 1 + isMoveFromEnemyExist;
	s_IS_MOVE_FROM_ENEMY = isMoveFromEnemyExist;
}

bool nxnGridGlobalActions::Step(State& s, double randomSelfAction, int action, OBS_TYPE lastObs, double& reward, OBS_TYPE& obs) const
{
	intVec state;
	nxnGridState::IdxToState(&s, state);

	// drawing more random numbers for each variable
	double randomSelfObservation = Random::RANDOM.NextDouble();

	std::vector<double> randomObjectMoves;
	CreateRandomVec(randomObjectMoves, CountMovingObjects() - 1);
	
	std::vector<double> randomEnemiesAttacks;
	CreateRandomVec(randomEnemiesAttacks, m_enemyVec.size());

	// run on all enemies and check if the robot was killed
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		if (CalcIfDead(i, state, randomEnemiesAttacks[i]))
		{
			reward = REWARD_LOSS;
			return true;
		}
	}

	// if we are at the target end game with a win
	if (m_targetIdx == state[0])
	{	
		reward = REWARD_WIN;
		return true;
	}

	reward = 0.0;
	// run on actions
	if (action == MOVE_TO_TARGET)
	{
		MoveToTarget(state, randomSelfAction);
	}
	else if(action == MOVE_TO_SHELTER & m_shelters.size() > 0)
	{
		MoveToShelter(state, randomSelfAction);
	}
	else // actions related to enemies
	{
		int enemyAction = (action - NumBasicActions()) % NumEnemyActions();
		int enemyIdx = (action - NumBasicActions()) / NumEnemyActions();
		++enemyIdx;

		if (enemyAction == ATTACK)
		{
			if (state[enemyIdx] != m_gridSize * m_gridSize)
			{
				Attack(state, enemyIdx, randomSelfAction, lastObs);
				int e = state[1];
				for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
				{
					if (state[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize)
					{
						reward = REWARD_KILL_NINV;
						return true;
					}
				}
				reward = REWARD_KILL_ENEMY * (state[enemyIdx] == m_gridSize * m_gridSize);
			}
			else
				reward = REWARD_ILLEGAL_MOVE;

		}
		else // action = movefromenemy
		{
			if (state[enemyIdx] != m_gridSize * m_gridSize)
				MoveFromEnemy(state, enemyIdx, randomSelfAction, lastObs);
			else
				reward = REWARD_ILLEGAL_MOVE;
		}
	}

	// set next position of the objects on grid
	SetNextPosition(state, randomObjectMoves);
	// update observation
	obs = FindObservation(state, randomSelfObservation);
	
	//update state
	nxnGridState& stateClass = static_cast<nxnGridState&>(s);
	stateClass.UpdateState(state);
	return false;
}

int nxnGridGlobalActions::NumBasicActions() const
{
	return 1 + (m_shelters.size() > 0);
}

int nxnGridGlobalActions::NumEnemyActions() const
{
	return 1 + s_IS_MOVE_FROM_ENEMY;
}

int nxnGridGlobalActions::NumActions() const
{
	// for any enemy add attack and move from enemy

	return NumBasicActions() + m_enemyVec.size() * NumEnemyActions();
};

ValuedAction nxnGridGlobalActions::GetMinRewardAction() const
{
	return ValuedAction(rand() % NumActions(), -10.0);
}

void nxnGridGlobalActions::PrintAction(int action, std::ostream & out) const
{
	out << s_ACTION_STR[action] << std::endl;
}

void nxnGridGlobalActions::AddActionsToEnemy()
{
	int enemyNum = m_enemyVec.size();
	s_ACTION_STR.push_back("Attack enemy #" + std::to_string(enemyNum));
	if (s_IS_MOVE_FROM_ENEMY)
		s_ACTION_STR.push_back("Move from enemy #" + std::to_string(enemyNum));
}

void nxnGridGlobalActions::AddActionsToShelter()
{
	// insert move to shelter after first action (move to target)
	if (m_shelters.size() == 1)
		s_ACTION_STR.insert(s_ACTION_STR.begin() + 1, "Move to shelter");

	s_numBasicActions = NumBasicActions();
}
bool nxnGridGlobalActions::MoveToTarget(intVec & state, double random) const
{
	Coordinate target(m_targetIdx % m_gridSize, m_targetIdx / m_gridSize);
	return MoveToLocation(state, target, random);
}

bool nxnGridGlobalActions::MoveToShelter(intVec & state, double random) const
{
	// go to the nearest shelter
	int shelterLoc = NearestShelter(state[0]);
	
	if (shelterLoc != state[0])
	{
		Coordinate shelter(shelterLoc % m_gridSize, shelterLoc / m_gridSize);
		return MoveToLocation(state, shelter, random);
	}

	return true;
}

void nxnGridGlobalActions::Attack(intVec & state, int idxEnemy, double random, OBS_TYPE lastObs) const
{
	intVec observedState;
	nxnGridState::IdxToState(lastObs, observedState);
	
	// if enemy was not observed do nothing, if enemy in range make attack o.w. move toward enemy
	if (observedState[0] == observedState[idxEnemy])
		return;
	else if (m_self.GetAttack()->InRange(state[0], state[idxEnemy], m_gridSize))
	{
		intVec shelters(m_shelters.size());
		for (size_t i = 0; i < m_shelters.size(); ++i)
			shelters[i] = m_shelters[i].GetLocation().GetIdx(m_gridSize);

		int attackLoc = observedState[idxEnemy];
		m_self.AttackOnline(state[0], attackLoc, state, shelters, m_gridSize, random);
	}
	else
	{
		Coordinate observedEnemy(observedState[idxEnemy] % m_gridSize, observedState[idxEnemy] / m_gridSize);
		MoveToLocation(state, observedEnemy, random);
	}
}

void nxnGridGlobalActions::MoveFromEnemy(intVec & state, int idxEnemy, double random, OBS_TYPE lastObs) const
{
	// if according to probability the robot is moving and the enemy is not dead move toward enemy
	if (random < m_self.GetSelfPMove())
	{
		intVec observedState; 
		nxnGridState::IdxToState(lastObs, observedState);
		// if enemy unobserved do nothing
		if (observedState[idxEnemy] != observedState[0])
		{
			Coordinate enemy(observedState[idxEnemy] % m_gridSize, observedState[idxEnemy] / m_gridSize);
			state[0] = MoveFromLocation(state, enemy);
		}
	}
}

bool nxnGridGlobalActions::MoveToLocation(intVec & state, Coordinate & goTo, double random) const
{
	int move = MoveToLocationIMP(state, goTo);

	random -= m_self.GetMovement().GetToward();
	if (random <= 0)
		state[0] = move;	

	return move != state[0];
}

int nxnGridGlobalActions::MoveFromLocation(intVec & state, Coordinate & goFrom) const
{
	Coordinate self(state[0] % m_gridSize, state[0] / m_gridSize);

	int maxLocation = state[0];
	int maxDist = self.Distance(goFrom);

	// run on all possible move locations and find the farthest point from goFrom
	for (size_t i = 0; i < s_numMoves; ++i)
	{
		Coordinate move(state[0] % m_gridSize, state[0] / m_gridSize);
		move.X() += s_lutDirections[i][0];
		move.Y() += s_lutDirections[i][1];

		// if move is not on grid continue for next move
		if (move.X() < 0 | move.X() >= m_gridSize | move.Y() < 0 | move.Y() >= m_gridSize)
			continue;

		// if move is valid calculate distance
		int currLocation = move.X() + move.Y() * m_gridSize;
		if (ValidLocation(state, currLocation))
		{
			int currDist = goFrom.Distance(move);
			if (currDist > maxDist)
			{
				maxLocation = currLocation;
				maxDist = currDist;
			}
		}
	}

	return maxLocation;
}

int nxnGridGlobalActions::MoveToLocationIMP(intVec & state, Coordinate & goTo) const
{
	int selfLocation = state[0];
	int goToLocation = goTo.X() + goTo.Y() * m_gridSize;
	int move = selfLocation;

	int xDiff = goTo.X() - selfLocation % m_gridSize;
	int yDiff = goTo.Y() - selfLocation / m_gridSize;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * m_gridSize : 0;

	// insert to move the best valid option

	// if the best move is valid return it
	move += changeToInsertX + changeToInsertY;
	if (ValidLocation(state, move))
		return move;

	// if the best move is not availabe compute the new best move and the second best move according to the distance between the available moves

	int secondMove;
	changeToInsertX += selfLocation;
	changeToInsertY += selfLocation;
	if (Distance(goToLocation, changeToInsertX, m_gridSize) > Distance(goToLocation, changeToInsertY, m_gridSize))
	{
		move = changeToInsertY;
		secondMove = changeToInsertX;
	}
	else
	{
		move = changeToInsertX;
		secondMove = changeToInsertY;
	}

	// return the best valid move
	if (ValidLocation(state, move))
		return move;
	else if (ValidLocation(state, secondMove))
		return secondMove;

	// if any move wasn't succesful return self location as no move
	return selfLocation;
}

int nxnGridGlobalActions::NearestShelter(int loc) const
{
	Coordinate location(loc % m_gridSize, loc / m_gridSize);
	int minDist = INT_MAX;
	int nearestShelter = -1;

	for (int s = 0; s < m_shelters.size(); ++s)
	{
		auto sLoc = m_shelters[s].GetLocation();
		int currDist = location.Distance(sLoc);
		if (currDist < minDist)
		{
			currDist = minDist;
			nearestShelter = sLoc.GetIdx(m_gridSize);
		}
	}

	return nearestShelter;
}

bool nxnGridGlobalActions::EnemyRelatedAction(int action) const
{
	return action >= NumBasicActions();
}

} //end ns despot