#include <string>
#include <math.h>

#include "..\include\despot\solver\pomcp.h"
#include "nxnGridLocalActions.h"
#include "Coordinate.h"

namespace despot 
{

/// changes surrounding a point
static const int s_numMoves = 8;
static const int s_lutDirections[s_numMoves][2] = { { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

/// string array for all actions (enum as idx)
static std::vector<std::string> s_ACTION_STR(nxnGridLocalActions::NUM_BASIC_ACTIONS + 1);

/// lut connecting coordinate change to move action
static std::vector<std::pair<int, int>> s_ACTION_CHANGE(nxnGridLocalActions::NUM_BASIC_ACTIONS);

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

nxnGridLocalActions::nxnGridLocalActions(int gridSize, int target, Self_Obj & self, std::vector<intVec> & objectsInitLoc)
: nxnGrid(gridSize, target, self, objectsInitLoc)
{
	// init vector for string of actions
	s_ACTION_STR[STAY] = "Stay";

	s_ACTION_STR[NORTH] = "North";
	s_ACTION_STR[SOUTH] = "South";
	s_ACTION_STR[WEST] = "West";
	s_ACTION_STR[EAST] = "East";

	s_ACTION_STR[NORTH_WEST] = "North-West";
	s_ACTION_STR[NORTH_EAST] = "North-East";
	s_ACTION_STR[SOUTH_WEST] = "South-West";
	s_ACTION_STR[SOUTH_EAST] = "South-East";

	s_ACTION_CHANGE[NORTH] = std::make_pair(0, -1);
	s_ACTION_CHANGE[SOUTH] = std::make_pair(0, 1);
	s_ACTION_CHANGE[WEST] = std::make_pair(-1, 0);
	s_ACTION_CHANGE[EAST] = std::make_pair(1, 0);

	s_ACTION_CHANGE[NORTH_WEST] = std::make_pair(-1, -1);
	s_ACTION_CHANGE[NORTH_EAST] = std::make_pair(1, -1);
	s_ACTION_CHANGE[SOUTH_WEST] = std::make_pair(-1, 1);
	s_ACTION_CHANGE[SOUTH_EAST] = std::make_pair(1, 1);

	s_numBasicActions = NUM_BASIC_ACTIONS;
	s_numEnemyRelatedActions = 1;
}

bool nxnGridLocalActions::Step(State& s, double randomSelfAction, int a, OBS_TYPE lastObs, double& reward, OBS_TYPE& obs) const
{
	intVec state;
	nxnGridState::IdxToState(&s, state);
	enum ACTION action = static_cast<enum ACTION>(a);

	// drawing more random numbers for each variable
	double randomSelfObservation = rand();
	randomSelfObservation /=  RAND_MAX;

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
	if (action > STAY)
	{
		if (action < NUM_BASIC_ACTIONS) // action == move
		{
			bool isValidMove = MakeMove(state, randomSelfAction, action);
			reward += REWARD_ILLEGAL_MOVE * (!isValidMove);
		}
		else // action = attack
		{
			int enemyIdx = action - NUM_BASIC_ACTIONS;
			if (state[enemyIdx] != m_gridSize * m_gridSize)
			{
				Attack(state, enemyIdx, randomSelfAction, obs);
				for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
				{
					if (state[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize)
					{
						reward = REWARD_LOSS;
						return true;
					}
				}
			}
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

int nxnGridLocalActions::NumActions() const
{
	// num basic actions + attack for each enemy
	return NUM_BASIC_ACTIONS + m_enemyVec.size();
};
void nxnGridLocalActions::PrintAction(int action, std::ostream & out) const
{
	out << s_ACTION_STR[action] << std::endl;
}

void nxnGridLocalActions::AddActionsToEnemy()
{
	s_ACTION_STR.push_back("Attack enemy #" + std::to_string(m_enemyVec.size()));
}

void nxnGridLocalActions::AddActionsToShelter()
{
}

bool nxnGridLocalActions::MakeMove(intVec & state, double random, ACTION action) const
{
	int x = state[0] % m_gridSize + s_ACTION_CHANGE[action].first;
	int y = state[0] / m_gridSize + s_ACTION_CHANGE[action].second;

	if (ValidLegalLocation(state, Coordinate(x, y), state.size(), m_gridSize))
	{
		state[0] = random < m_self.GetSelfPMove() ? x + y * m_gridSize : state[0];
		return true;
	}

	return false;
}
void nxnGridLocalActions::Attack(intVec & state, int enemyIdx, double random, OBS_TYPE lastObs) const
{
	intVec observedState; 
	nxnGridState::IdxToState(lastObs, observedState);
	
	// if enemy is not observed do nothing
	if (observedState[enemyIdx] == observedState[0])
		return;
	if (m_self.GetObservation()->InRange(state[0], observedState[enemyIdx], m_gridSize))
	{
		intVec shelters(m_shelters.size());
		for (size_t i = 0; i < m_shelters.size(); ++i)
			shelters[i] = m_shelters[i].GetLocation().GetIdx(m_gridSize);

		int attackLoc = observedState[enemyIdx];
		m_self.AttackOnline(state[0], attackLoc, state, shelters, m_gridSize, random);
	}
	else
	{
		Coordinate observedEnemy(observedState[1] % m_gridSize, observedState[enemyIdx] / m_gridSize);
		MoveToLocation(state, observedEnemy, random);
	}
}

void nxnGridLocalActions::MoveToLocation(intVec & state, Coordinate & goTo, double random) const
{
	int move = MoveToLocationIMP(state, goTo);

	random -= m_self.GetMovement().GetToward();
	if (random <= 0)
		state[0] = move;	
}

int nxnGridLocalActions::MoveToLocationIMP(intVec & state, Coordinate & goTo) const
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

bool nxnGridLocalActions::EnemyRelatedAction(int action) const
{
	return action >= NUM_BASIC_ACTIONS;
}

} //end ns despot