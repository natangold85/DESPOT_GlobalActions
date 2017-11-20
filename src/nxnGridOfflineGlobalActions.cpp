#include "nxnGridOfflineGlobalActions.h"

#include <iostream>
static const std::string s_WinState = "Win";
static const std::string s_LossState = "Loss";

/// value in move states for non-valid move
static const int NVALID_MOVE = -1;
/// all directions which are also all possible moves
static const int s_NUM_DIRECTIONS = 8;
/// lut for all direction changes
static int s_lutDirections[s_NUM_DIRECTIONS][2] = { { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;

	return xDiff * xDiff + yDiff * yDiff;
}

inline int Abs(int x)
{
	return x * (x >= 0) - x * (x < 0);
}

nxnGridOfflineGlobalActions::nxnGridOfflineGlobalActions(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount)
: nxnGridOffline(gridSize, targetIdx, self, isFullyObs, discount)
{
}

void nxnGridOfflineGlobalActions::SaveInPomdpFormat(FILE *fptr)
{
	std::string buffer("");
	//add comments and init lines(state observations etc.) to file
	CommentsAndInitLines(fptr);

	// add position with and without moving of the robot
	AddAllActions(fptr);

	// add observations and rewards
	ObservationsAndRewards(fptr);
}

int nxnGridOfflineGlobalActions::GetNumActions() const
{
	return 1 + (m_shelterVec.size() != 0) + m_enemyVec.size() * NUM_ENEMY_ACTIONS;
}

void nxnGridOfflineGlobalActions::CommentsAndInitLines(FILE *fptr)
{
	std::string buffer;

	// add comments
	buffer += "# pomdp file:\n";
	buffer += "# grid size: " + std::to_string(m_gridSize) + "  target idx: " + std::to_string(m_targetIdx);
	buffer += "\n# SELF:\n# self initial location: " + std::to_string(m_self.GetLocation().GetIdx(m_gridSize));
	buffer += "\n# with attack: " + m_self.GetAttack()->String();
	buffer += "\n# with observation: " + m_self.GetObservation()->String();
	buffer += "\n\n# ENEMIES:";
	for (auto v : m_enemyVec)
	{
		buffer += "\n\n# enemy initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
		buffer += "\n# p(toward target) = " + std::to_string(v.GetMovement().GetToward()) + " p(stay) = " + std::to_string(v.GetMovement().GetStay());
		buffer += "\n# with attack: " + v.GetAttack()->String();
	}

	buffer += "\n\n# NON-INVOLVED:";
	for (auto v : m_nonInvolvedVec)
	{
		buffer += "\n\n# non- involved initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
		buffer += "\n# p(toward target) = " + std::to_string(v.GetMovement().GetToward()) + " p(stay) = " + std::to_string(v.GetMovement().GetStay());
	}

	buffer += "\n\n# SHELTERS:";
	for (auto v : m_shelterVec)
	{
		buffer += "\n# shelter location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
	}

	// add init lines
	buffer += "\n\ndiscount: " + std::to_string(m_discount);
	buffer += "\nvalues: reward\nstates: ";

	// add states names
	CalcStatesAndObs("s", buffer);
	buffer += s_WinState + " " + s_LossState + "\n";
	buffer += "actions: MoveToTarget ";
	if (m_shelterVec.size() > 0)
		buffer += "MoveToShelter";

	for (int e = 0; e < m_enemyVec.size(); ++e)
	{
		buffer += " Attack" + std::to_string(e);
		buffer += " MoveFromEnemy" + std::to_string(e);
	}
	buffer += "\n";

	// add observations names
	buffer += "observations: ";
	CalcStatesAndObs("o", buffer);
	buffer += "o" + s_WinState + " o" + s_LossState + "\n";
	buffer += "\n\nstart: \n";

	// add start states probability
	CalcStartState(buffer);
	buffer += "\n\n";
	//save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void nxnGridOfflineGlobalActions::AddAllActions(FILE * fptr)
{
	std::string buffer;

	// add move to win state from states when the robot is in target
	intVec state(CountMovableObj());
	state[0] = m_targetIdx;
	AddTargetPositionRec(state, 1, buffer);
	buffer += "\n";

	// add actions for all states
	AddActionsAllStates(buffer);

	// save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void nxnGridOfflineGlobalActions::AddActionsAllStates(std::string & buffer)
{
	intVec state(CountMovableObj());
	std::string action = "*";

	std::vector<int> shelters;
	CreateShleterVec(shelters);

	AddActionsRec(state, shelters, 0, buffer);

	buffer += "\n\nT: * : " + s_WinState + " : " + s_WinState + " 1";
	buffer += "\nT: * : " + s_LossState + " : " + s_LossState + " 1\n\n";
}

void nxnGridOfflineGlobalActions::AddActionsRec(intVec & state, intVec & shelters, int currObj, std::string & buffer)
{
	if (currObj == state.size())
	{
		// if we are allready in the target idx do nothing
		if (state[0] == m_targetIdx)
			return;

		//int m = state[0];
		//int e1 = state[1];
		//int e2 = state[2];

		AddMoveToTarget(state, shelters, buffer);
		
		// if exist shelter add move to shelter action
		if (m_shelterVec.size() > 0)
			AddMoveToShelter(state, shelters, buffer);

		for (int e = 0; e < m_enemyVec.size(); ++e)
		{
			AddAttack(state, e, shelters, buffer);
			AddMoveFromEnemy(state, e, shelters, buffer);
		}
	}
	else
	{
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			state[currObj] = i;
			if (NoRepeats(state, currObj))
			{
				AddActionsRec(state, shelters, currObj + 1, buffer);
			}
		}
		// if the object is enemy add cases of when the enemy is dead
		if (IsEnemy(currObj))
		{
			state[currObj] = m_gridSize * m_gridSize;
			AddActionsRec(state, shelters, currObj + 1, buffer);
		}
	}
}

void nxnGridOfflineGlobalActions::AddAttack(intVec & state, int enemyIdx, intVec & shelters, std::string & buffer) const
{
	std::string action = "Attack" + std::to_string(enemyIdx);
	
	s_pLeftProbability = 1.0;
	double pLoss = 0.0;

	// if enemy dead do nothing
	if (state[enemyIdx + 1] == m_gridSize * m_gridSize)
	{
		pLoss = PositionSingleState(state, state, shelters, action, buffer);
		if (pLoss > 0.0)
			buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";
		buffer += "\n";
		return;
	}

	Coordinate self(state[0] % m_gridSize, state[0] / m_gridSize);
	Coordinate enemy(state[enemyIdx + 1] % m_gridSize, state[enemyIdx + 1] / m_gridSize);

	if (m_self.GetAttack()->GetRange() >= self.RealDistance(enemy))
	{
		std::vector<std::pair<intVec, double>> shootOutcomes;

		// creating a vecotr of shoot outcomes
		m_self.AttackOffline(state[0], state[enemyIdx + 1], state, shelters, m_gridSize, shootOutcomes);

		for (auto v : shootOutcomes)
		{
			// if non-involved dead transfer to loss state else calculate move states
			if (!IsNonInvDead(v.first))
			{
				s_pLeftProbability = v.second;
				pLoss += PositionSingleState(v.first, state, shelters, action, buffer);
			}
			else
				pLoss += v.second;
		}
	}
	else
		pLoss += MoveToLocation(state, shelters, state[enemyIdx + 1], action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineGlobalActions::AddMoveToTarget(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveToTarget";
	
	s_pLeftProbability = 1.0;
	double pLoss = 0.0;

	std::vector<std::pair<int, double>> moveOutComes;
	pLoss += MoveToLocation(state, shelters, m_targetIdx, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineGlobalActions::AddMoveToShelter(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveToShelter";
	
	double pLoss = 0.0;
	s_pLeftProbability = 1.0;

	if (m_shelterVec.size() > 0 && !SearchForShelter(state[0]))
	{
		int shelterLoc = FindNearestShelter(state[0]);
		pLoss += MoveToLocation(state, shelters, shelterLoc, action, buffer);
	}
	else
		pLoss += PositionSingleState(state, state, shelters, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineGlobalActions::AddMoveFromEnemy(intVec & state, int idxEnemy, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveFromEnemy" + std::to_string(idxEnemy);

	double pLoss = 0.0;
	s_pLeftProbability = 1.0;

	// if enemy dead or not exist do nothing
	if (state[idxEnemy + 1] == m_gridSize * m_gridSize)
	{
		pLoss = PositionSingleState(state, state, shelters, action, buffer);
		if (pLoss > 0.0)
			buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";
		buffer += "\n";
		return;
	}

	int newLoc = MoveFromLocation(state, state[idxEnemy + 1]);

	intVec newState(state);

	// if current location of self is not the farthest point from the enemy make move
	if (newLoc != state[0])
	{
		s_pLeftProbability = 1 - m_self.GetSelfPMove();
		pLoss += PositionSingleState(state, state, shelters, action, buffer);

		intVec newState(state);
		newState[0] = newLoc;
		s_pLeftProbability = m_self.GetSelfPMove();
		pLoss += PositionSingleState(newState, state, shelters, action, buffer);
	}
	else
		pLoss += PositionSingleState(state, state, shelters, action, buffer);


	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

double nxnGridOfflineGlobalActions::MoveToLocation(intVec & state, intVec & shelters, int location, std::string & action, std::string & buffer) const
{
	std::pair<double, double> goTo = std::make_pair(location % m_gridSize, location / m_gridSize);

	int newLoc = MoveToLocationIMP(state, location);

	intVec newState(state);
	double pLoss = 0.0;

	if (newLoc >= 0)
	{
		s_pLeftProbability = 1 - m_self.GetSelfPMove();
		pLoss += PositionSingleState(state, state, shelters, action, buffer);

		intVec newState(state);
		newState[0] = newLoc;
		s_pLeftProbability = m_self.GetSelfPMove();
		pLoss += PositionSingleState(newState, state, shelters, action, buffer);
	}
	else
		pLoss += PositionSingleState(state, state, shelters, action, buffer);

	return pLoss;
}

int nxnGridOfflineGlobalActions::MoveFromLocation(intVec & state, int fromLocation) const
{
	std::pair<int, int> self = std::make_pair(state[0] % m_gridSize, state[0] / m_gridSize);

	int maxLocation = state[0];
	int maxDist = Distance(fromLocation, state[0], m_gridSize);

	// run on all possible move locations and find the farthest point from fromLocation
	for (size_t i = 0; i < s_NUM_DIRECTIONS; ++i)
	{
		std::pair<int, int> move = std::make_pair(state[0] % m_gridSize, state[0] / m_gridSize);
		move.first += s_lutDirections[i][0];
		move.second += s_lutDirections[i][1];

		// if move is not on grid continue for next move
		if (move.first < 0 | move.first >= m_gridSize | move.second < 0 | move.second >= m_gridSize)
			continue;

		// if move is valid calculate distance
		int currLocation = move.first + move.second * m_gridSize;
		if (NoRepeatsLocation(state, currLocation))
		{
			int currDist = Distance(fromLocation, currLocation, m_gridSize);
			// insert location to maxlocation if curr distance is max dist
			if (currDist > maxDist)
			{
				maxLocation = currLocation;
				maxDist = currDist;
			}

		}
	}
	return maxLocation;
}

int nxnGridOfflineGlobalActions::MoveToLocationIMP(intVec & state, int goTo) const
{
	int selfLocation = state[0];

	int xDiff = goTo % m_gridSize - selfLocation % m_gridSize;
	int yDiff = goTo / m_gridSize - selfLocation / m_gridSize;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * m_gridSize : 0;

	int move = selfLocation + changeToInsertX + changeToInsertY;

	// if the best move is valid return it else if there is only one direction to advance return -1
	if (NoRepeatsLocation(state, move))
		return move;
	else if (changeToInsertX == 0 | changeToInsertY == 0)
		return -1;

	// try move to in the axis in which we are farther than goTo
	int secondMove;
	if (Distance(goTo, selfLocation + changeToInsertX, m_gridSize) < Distance(goTo, selfLocation + changeToInsertY, m_gridSize))
	{
		move = selfLocation + changeToInsertX;
		secondMove = selfLocation + changeToInsertY;
	}
	else
	{
		secondMove = selfLocation + changeToInsertX;
		move = selfLocation + changeToInsertY;
	}

	if (NoRepeatsLocation(state, move))
		return move;

	if (NoRepeatsLocation(state, secondMove))
		return secondMove;

	return -1;
}
