#include <iostream>		// cout
#include <string>		// string
#include <sstream>		// ostringstream
#include <iomanip>		// set_percision
#include <algorithm>	// algorithms
#include <fstream>      // std::ofstream

#include "POMDP_Writer.h"

inline void READ(std::ifstream & in, char *toRead, int size)
{
	if (!in.read(toRead, size))
	{
		std::cerr << "error in reading\n";
		exit(1);
	}
}

static const std::string s_WinState = "Win";
static const std::string s_LossState = "Loss";

/// value in move states for non-valid move
static const int NVALID_MOVE = -1;
/// all directions which are also all possible moves
static const int s_NUM_DIRECTIONS = 8;
/// lut for all direction changes
static int s_lutDirections[s_NUM_DIRECTIONS][2] = { { 0,1 },{ 0,-1 },{ 1,0 },{ -1,0 },{ 1,1 },{ 1,-1 },{ -1,1 },{ -1,-1 } };

/// number of moves of an object from a specific location (stay, num moves, moving toward robot)
static const int s_NUM_OPTIONS_MOVE = s_NUM_DIRECTIONS + 2;

/// to convey probability between calculations
static double s_pLeftProbability = 1.0;


inline int Abs(int x)
{
	return x * (x >= 0) - x * (x < 0);
}

inline void Swap(int & a, int & b)
{
	int tmp = a;
	a = b;
	b = tmp;
}

inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;

	return xDiff * xDiff + yDiff * yDiff;
}

static double CalcPAUB(std::vector<double> & pVec)
{
	if (pVec.size() == 1)
		return pVec[0];
	else if (pVec.size() == 2)
		return pVec[0] + pVec[1] - pVec[0] * pVec[1];
	else if (pVec.size() == 3)
		return pVec[0] + pVec[1] + pVec[2] - pVec[0] * pVec[1] - pVec[1] * pVec[2] - pVec[0] * pVec[2] + pVec[0] * pVec[1] * pVec[2];
	// need to continue if i want to allow more than 3 enemies
}

// translate to string with higher precision
inline std::string to_string_precision(double d, int n = 10)
{
	std::ostringstream out;
	out << std::setprecision(n) << d;
	return out.str();
}

POMDP_Writer::POMDP_Writer(int gridSize, int targetIdx, Self_Obj& self, double discount)
	: m_gridSize(gridSize)
	, m_targetIdx(targetIdx)
	, m_discount(discount)
	, m_self(self)
	, m_enemyVec()
	, m_nonInvolvedVec()
	, m_shelterVec()
{
}


void POMDP_Writer::UpdateSelf(Self_Obj && obj)
{
	m_self = obj;
}

void POMDP_Writer::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
}

void POMDP_Writer::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));
}

void POMDP_Writer::AddObj(ObjInGrid&& obj)
{
	m_shelterVec.emplace_back(std::forward<ObjInGrid>(obj));
}

void POMDP_Writer::SetLocationSelf(Point & newLocation)
{
	m_self.SetLocation( newLocation );
}

void POMDP_Writer::SetLocationEnemy(Point & newLocation, int idxObj)
{

	m_enemyVec[idxObj - 1].SetLocation(newLocation);
}

void POMDP_Writer::SetLocationNonInv(Point & newLocation, int idxObj)
{
	m_nonInvolvedVec[idxObj - 1 - m_enemyVec.size()].SetLocation(newLocation);
}

void POMDP_Writer::SetLocationShelter(Point & newLocation, int idxObj)
{
	m_shelterVec[idxObj - 1 - m_enemyVec.size() - m_nonInvolvedVec.size()].SetLocation(newLocation);
}

void POMDP_Writer::SetTarget(int idx)
{
	m_targetIdx = idx;
}

void POMDP_Writer::SetGridSize(int gridSize)
{
	m_gridSize = gridSize;
}

void POMDP_Writer::SaveInFormat(FILE *fptr)
{
	std::string buffer("");
	//add comments and init lines(state observations etc.) to file
	CommentsAndInitLines(fptr);

	// add position with and without moving of the robot
	AddAllActions(fptr);

	// add observations and rewards
	ObservationsAndRewards(fptr);
}


void POMDP_Writer::CommentsAndInitLines(FILE *fptr)
{
	std::string buffer;

	// add comments
	buffer += "# pomdp file:\n";
	buffer += "# grid size: " + std::to_string(m_gridSize) + "  target idx: " + std::to_string(m_targetIdx);
	buffer += "\n# SELF:\n# self initial location: " + std::to_string(m_self.GetLocation().GetIdx(m_gridSize)) + " std = " + std::to_string(m_self.GetLocation().GetStd());
	buffer += "\n# with atack range of " + std::to_string(m_self.GetRange()) + " (p = " + std::to_string(m_self.GetPHit()) + ")";
	buffer += "\n# observation range of " + std::to_string(m_self.GetRangeObs()) + " (p = " + std::to_string(m_self.GetPObs()) + ")";
	buffer += "\n\n# ENEMIES:";
	for (auto v : m_enemyVec)
	{
		buffer += "\n\n# enemy initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize)) + " std = " + std::to_string(v.GetLocation().GetStd());
		buffer += "\n# p(toward target) = " + std::to_string(v.GetMovement().GetToward()) + " p(stay) = " + std::to_string(v.GetMovement().GetStay());
		buffer += "\n# with atack range of " + std::to_string(m_self.GetRange()) + " (p = " + std::to_string(m_self.GetPHit()) + ") ";
	}

	buffer += "\n\n# NON-INVOLVED:";
	for (auto v : m_nonInvolvedVec)
	{
		buffer += "\n\n# non- involved initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize)) + " std = " + std::to_string(v.GetLocation().GetStd());
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
	buffer += "actions: MoveToTarget MoveToShelter Attack MoveFromEnemy\n";

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

void POMDP_Writer::AddAllActions(FILE * fptr)
{
	std::string buffer;
	//buffer += "\n\nT: * : * : * 0.0\n\n";
	
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

void POMDP_Writer::ObservationsAndRewards(FILE * fptr)
{
	std::string buffer;
	// calculate observations
	CalcObs(buffer);
	// add rewards
	buffer += "\n\nR: * : * : * : * 0.0\nR: * : "
		+ s_WinState + " : * : * 1\nR: * : "
		+ s_LossState + " : * : * -2\n";
	// save to file 
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; return; }
}

void POMDP_Writer::CalcStatesAndObs(const char * type, std::string& buffer)
{
	intVec state(CountMovableObj());
	CalcS_ORec(state, 0, type, buffer);
}

void POMDP_Writer::CalcS_ORec(intVec& state, int currIdx, const char * type, std::string & buffer)
{
	// stopping condition when finish running on all objects
	if (state.size() == currIdx)
	{
		// insert state to buffer
		buffer += GetStringState(state, type) + " ";
	}
	else
	{
		// run on all possible states (without repetitions)
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			state[currIdx] = i;
			if (NoRepeats(state, currIdx))
			{
				CalcS_ORec(state, currIdx + 1, type, buffer);
			}
		}

		if (IsEnemy(currIdx))
		{
			state[currIdx] = m_gridSize * m_gridSize;
			CalcS_ORec(state, currIdx + 1, type, buffer);
		}
	}
}

bool POMDP_Writer::NoRepetitionCheckAndCorrect(intVec& state, std::vector<intVec> & moveStates, intVec & arrOfIdx) const
{
	//if any move state equal to the robot location change location to previous location
	for (int i = 1; i < state.size(); ++i)
	{
		if (state[i] == state[0])
		{
			state[i] = moveStates[i - 1][0];
		}
	}

	//if one of the state equal to another return the possible state to the previous location
	for (int i = 1; i < state.size(); ++i)
	{
		for (int j = 1; j < state.size(); ++j)
		{
			if (state[i] == state[j] && i != j && state[j] != m_gridSize * m_gridSize)
			{
				if (arrOfIdx[i - 1] == 0)
				{
					state[j] = moveStates[(j - 1)][0];
					arrOfIdx[j - 1] = 0;
					return false;
				}
				else
				{
					state[i] = moveStates[(i - 1)][0];
					arrOfIdx[i - 1] = 0;
					return false;
				}
			}
		}
	}

	return true;
}

bool POMDP_Writer::NoRepeatsLocation(intVec& state, int loc) const
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (state[i] == loc)
		{
			return false;
		}
	}
	return true;
}

bool POMDP_Writer::NoRepeatsAll(intVec& state) const
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (!NoRepeats(state,i))
		{
			return false;
		}
	}
	return true;
}

bool POMDP_Writer::NoRepeats(intVec& state, int idx) const
{
	if (state[idx] == m_gridSize * m_gridSize)
	{
		return true;
	}

	for (int i = 0; i < idx; ++i)
	{
		if (state[i] == state[idx])
		{
			return false;
		}
	}
	return true;
}

void POMDP_Writer::CalcStartState(std::string& buffer)
{
	intVec state(CountMovableObj());

	// create init state
	intVec initState;
	initState.emplace_back(m_self.GetLocation().GetIdx(m_gridSize));
	for (int i = 0; i < m_enemyVec.size(); ++i)
		initState.emplace_back(m_enemyVec[i].GetLocation().GetIdx(m_gridSize));
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
		initState.emplace_back(m_nonInvolvedVec[i].GetLocation().GetIdx(m_gridSize));
	
	// calculate probability for each state
	CalcStartStateRec(state, initState, 0, buffer);

	// add probability of win and loss states
	buffer += "0 0 ";
}

void POMDP_Writer::CalcStartStateRec(intVec& state, intVec& initState, int currIdx, std::string & buffer)
{
	// stopping condition when finish running on all objects
	if (state.size() == currIdx)
	{
		// insert prob to init to buffer
		if (state == initState)
			buffer += "1 ";
		else
			buffer += "0 ";
	}
	else
	{
		// run on all possible locations and if the location is valid run on the next object
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			state[currIdx] = i;
			if (NoRepeats(state, currIdx))
			{
				CalcStartStateRec(state, initState, currIdx + 1, buffer);
			}
		}
		
		// add the option of dead enemy if the object is enbemy
		if (IsEnemy(currIdx))
		{
			state[currIdx] = m_gridSize * m_gridSize;
			CalcStartStateRec(state, initState, currIdx + 1, buffer);
		}
	}
}

void POMDP_Writer::AddTargetPositionRec(intVec & state, int currIdx, std::string & buffer)
{
	if (currIdx == state.size())
	{
		buffer += "T: * : " + GetStringState(state) + " : " + s_WinState + " 1.0000\n";
	}
	else
	{
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			state[currIdx] = i;
			if (NoRepeats(state, currIdx))
				AddTargetPositionRec(state, currIdx + 1, buffer);
		}
		if (IsEnemy(currIdx))
		{
			state[currIdx] = m_gridSize * m_gridSize;
			AddTargetPositionRec(state, currIdx + 1, buffer);
		}
	}

}

void POMDP_Writer::AddActionsAllStates(std::string & buffer)
{
	intVec state(CountMovableObj());
	std::string action = "*";

	std::vector<int> shelters;
	CreateShleterVec(shelters);

	AddActionsRec(state, shelters, 0, buffer);

	StartAgain(buffer, s_WinState);
	StartAgain(buffer, s_LossState);
	buffer += "\n";
}


void POMDP_Writer::StartAgain(std::string & buffer, const std::string & terminatingState)
{
	// assuming initial condition is fixed!!
	intVec initState(CountMovableObj());

	initState[0] = m_self.GetLocation().GetIdx(m_gridSize);

	for (size_t i = 0; i < m_enemyVec.size(); ++i)
		initState[i +1] = m_enemyVec[i].GetLocation().GetIdx(m_gridSize);

	for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
		initState[i + 1 + m_enemyVec.size()] = m_nonInvolvedVec[i].GetLocation().GetIdx(m_gridSize);

	buffer += "T: * : " + terminatingState + " : " + GetStringState(initState) + " 1.0000\n";
}

void POMDP_Writer::AddActionsRec(intVec & state, intVec & shelters, int currObj, std::string & buffer)
{
	if (currObj == state.size())
	{
		// if we are allready in the target idx do nothing
		if (state[0] == m_targetIdx)
			return;
		
		AddAttack(state, shelters, buffer);
		AddMoveToTarget(state, shelters, buffer);
		AddMoveToShelter(state, shelters, buffer);
		AddMoveFromEnemy(state, shelters, buffer);
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

void POMDP_Writer::AddAttack(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "Attack";
	double pLoss = 0.0;

	// if enemy dead or not exist do nothing
	if (m_enemyVec.size() == 0 || state[1] == m_gridSize * m_gridSize)
	{
		PositionSingleState(state, state, shelters, action, buffer);
		buffer += "\n";
		return;
	}

	if (m_self.IsInRange(state[0], state[1], m_gridSize))
	{
		std::vector<std::pair<intVec, double>> shootOutcomes;

		// creating a vecotr of shoot outcomes
		m_self.CalcSelfAttackSARSOP(state, shelters, m_gridSize, shootOutcomes);

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
	{
		pLoss += MoveToLocation(state, shelters, state[1], action, buffer);
	}

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + std::to_string(pLoss) + "\n";

	s_pLeftProbability = 1.0;
	buffer += "\n";
}

void POMDP_Writer::AddMoveToTarget(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveToTarget";
	double pLoss = 0.0;

	std::vector<std::pair<int, double>> moveOutComes;
	pLoss += MoveToLocation(state, shelters, m_targetIdx, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + std::to_string(pLoss) + "\n";

	s_pLeftProbability = 1.0;
	buffer += "\n";
}

void POMDP_Writer::AddMoveToShelter(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveToShelter";
	double pLoss = 0.0;

	if (m_shelterVec.size() > 0 && !SearchForShelter(state[0]))
	{
		int shelterLoc = FindNearestShelter(state[0]);
		pLoss += MoveToLocation(state, shelters, shelterLoc, action, buffer);
	}
	else
		pLoss += PositionSingleState(state, state, shelters, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + std::to_string(pLoss) + "\n";
	
	s_pLeftProbability = 1.0;
	buffer += "\n";
}

void POMDP_Writer::AddMoveFromEnemy(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveFromEnemy";

	// if enemy dead or not exist do nothing
	if (m_enemyVec.size() == 0 || state[1] == m_gridSize * m_gridSize)
	{
		PositionSingleState(state, state, shelters, action, buffer);
		buffer += "\n";
		return;
	}

	double pLoss = 0.0;

	int newLoc = MoveFromLocation(state, state[1]);

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
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + std::to_string(pLoss) + "\n";

	s_pLeftProbability = 1.0;
	buffer += "\n";
}

void POMDP_Writer::CreateShleterVec(intVec & shelters) const
{
	for (auto v : m_shelterVec)
		shelters.push_back(v.GetLocation().GetIdx(m_gridSize));
}

bool POMDP_Writer::IsNonInvDead(intVec & state) const
{
	bool isDead = false;

	// searching for dead non-involved
	for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
		isDead |= (state[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize);

	return isDead;
}

double POMDP_Writer::MoveToLocation(intVec & state, intVec & shelters, int location, std::string & action, std::string & buffer) const
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

int POMDP_Writer::MoveFromLocation(intVec & state, int fromLocation) const
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

int POMDP_Writer::MoveToLocationIMP(intVec & state, int goTo) const
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
	if (Distance(goTo, selfLocation + changeToInsertX, m_gridSize) > Distance(goTo, selfLocation + changeToInsertY, m_gridSize))
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

int POMDP_Writer::FindNearestShelter(int location) const
{
	// assumption : support only 1 shelter
	return m_shelterVec[0].GetLocation().GetIdx(m_gridSize);
}

double POMDP_Writer::PositionSingleState(intVec & newState, intVec & currentState, intVec & shelters, std::string & action, std::string & buffer) const
{
	std::string prefix = "T: " + action + " : " + GetStringState(currentState) + " : ";

	double pToDead = 0.0;

	// if the enemy is not dead calculate his attack (not support more than 1 enemy so far)
	if (m_enemyVec.size() > 0 && currentState[1] != m_gridSize * m_gridSize)
		m_enemyVec[0].CalcEnemyAttackSARSOP(currentState, shelters, m_gridSize, pToDead);

	// if ptoDead is not 0 calculate new whole probability and return pToDead * pLeftProb
	if (pToDead > 0.0)
	{
		double remember = s_pLeftProbability;
		s_pLeftProbability *= 1 - pToDead;
		pToDead *= remember;
	}

	prefix += "s";

	std::vector<intVec> moveStates(CountMovableObj() - 1);
	for (int i = 0; i < moveStates.size(); ++i)
		moveStates[i].resize(s_NUM_OPTIONS_MOVE);

	CalcMoveStates(newState, moveStates);

	// array of idx pointing to the current move state
	std::vector<int> arrOfIdx(CountMovableObj() - 1);
	for (int i = 0; i < CountMovableObj() - 1; ++i)
	{
		arrOfIdx[i] = 0;
	}

	mapProb pMap;
	// calculate the probability of each move state and insert it to pMap
	AddMoveStatesRec(newState, moveStates, arrOfIdx, 0, pMap);
	
	// insert the move states to the buffer
	int numStates = m_gridSize * m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, numStates](pairMap itr)
	{	buffer += prefix;	AddStateToBuffer(buffer, itr, numStates); });

	return pToDead;
}

void POMDP_Writer::AddStateToBuffer(std::string& buffer, pairMap itr, int numLocations)
{
	buffer += std::to_string(itr.first[0]);

	for (int i = 1; i < itr.first.size(); ++i)
	{
		// if object is dead insert dead to buffer
		if (itr.first[i] == numLocations)
		{
			buffer += "xD";
		}
		else
		{
			buffer += "x" + std::to_string(itr.first[i]);
		}
	}
	buffer += " " + to_string_precision(itr.second) + "\n";
}

std::string POMDP_Writer::GetStringState(intVec & state) const
{
	return GetStringState(state, "s");
}

std::string POMDP_Writer::GetStringState(intVec & state, const char *type) const
{
	std::string currentState = type + std::to_string(state[0]);

	for (int i = 1; i < state.size(); ++i)
	{
		if (state[i] != m_gridSize * m_gridSize)
		{
			currentState += "x" + std::to_string(state[i]);
		}
		else
		{
			currentState += "xD";
		}
	}
	return std::move(currentState);
}

bool POMDP_Writer::SearchForShelter(int location) const
{
	for (auto v : m_shelterVec)
	{
		if (v.GetLocation().GetIdx(m_gridSize) == location)
		{
			return true;
		}
	}
	return false;
}

bool POMDP_Writer::InBoundary(int state, int advanceFactor, int gridSize)
{
	int x = state % gridSize;
	int y = state / gridSize;
	int xdiff = advanceFactor % gridSize;
	int ydiff = advanceFactor / gridSize;

	return ((x + xdiff) >= 0) & ((x + xdiff) < gridSize) & ((y + ydiff) >= 0) & ((y + ydiff) < gridSize);
}

inline bool POMDP_Writer::InBoundary(int location, int xChange, int yChange, int gridSize)
{
	int x = location % gridSize + xChange;
	int y = location / gridSize + yChange;
	// return true if the location is in boundary
	return x >= 0 & x < gridSize & y >= 0 & y < gridSize;
}

void POMDP_Writer::AddMoveStatesRec(intVec & state, std::vector<intVec> & moveStates, intVec & arrOfIdx, int currIdx, mapProb & pMap) const
{
	if (currIdx == state.size() - 1)
	{
		intVec s = MoveToIdx(state, moveStates, arrOfIdx);
		pMap[MoveToIdx(state, moveStates, arrOfIdx)] += CalcProb2Move(arrOfIdx);
	}
	else
	{	
		for (int i = 0; i < s_NUM_OPTIONS_MOVE; ++i)
		{
			arrOfIdx[currIdx] = i;
			AddMoveStatesRec(state, moveStates, arrOfIdx, currIdx + 1, pMap);
		}
	}
}


POMDP_Writer::intVec POMDP_Writer::MoveToIdx(intVec state, std::vector<intVec> & moveStates, intVec & arrOfIdx) const
{
	// insert the current moveState to state
	for (int i = 0; i < state.size() - 1; ++i)
	{
		int currState = moveStates[i][arrOfIdx[i]];
		state[i + 1] = currState * (NVALID_MOVE != currState) + moveStates[i][0] * (NVALID_MOVE == currState);
	}

	// correct the state vec in case of repetitions
	while (!NoRepetitionCheckAndCorrect(state, moveStates, arrOfIdx));

	return state;
}

double POMDP_Writer::CalcProb2Move(intVec & arrOfIdx) const
{
	// insert to p the whole (replacement of 1 due to cases that reduce probability)
	double pMoveState = s_pLeftProbability;
	
	// multiply with the p(enemy = moveState)
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		int relativeIdx = arrOfIdx[i] % s_NUM_OPTIONS_MOVE;

		pMoveState *= m_enemyVec[i].GetMovement().GetStay() * (relativeIdx == 0) +
			m_enemyVec[i].GetMovement().GetEqual() * (relativeIdx > 0 & relativeIdx < s_NUM_OPTIONS_MOVE - 1) +
			m_enemyVec[i].GetMovement().GetToward() * (relativeIdx == s_NUM_OPTIONS_MOVE - 1);
	}
	// multiply with the p(non-involved = moveState)
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		int relativeIdx = arrOfIdx[i + m_enemyVec.size()] % s_NUM_OPTIONS_MOVE;
		pMoveState *= m_nonInvolvedVec[i].GetMovement().GetStay() * (relativeIdx == 0) +
			m_nonInvolvedVec[i].GetMovement().GetEqual() * (relativeIdx > 0 & relativeIdx < s_NUM_OPTIONS_MOVE - 1) +
			m_nonInvolvedVec[i].GetMovement().GetToward() * (relativeIdx == s_NUM_OPTIONS_MOVE - 1);
	}

	return pMoveState;
}

void POMDP_Writer::CalcMoveStates(intVec & state, std::vector<intVec> & moveStates) const
{
	// calculate possible move state (for non-valid move state insert -1 in movestates array)
	for (int i = 0; i < state.size() - 1; ++i)
	{
		// insert stay option
		moveStates[i][0] = state[i + 1];
		if (state[i + 1] != m_gridSize * m_gridSize)
		{
			int x = state[i + 1] % m_gridSize;
			int y = state[i + 1] / m_gridSize;

			int minDist = Distance(state[i + 1], state[0], m_gridSize);
			int minIdx = -1;
			for (int j = 0; j < s_NUM_DIRECTIONS; ++j)
			{
				int newX = x + s_lutDirections[j][0];
				int newY = y + s_lutDirections[j][1];
				bool isValid = (newX >= 0) & (newX < m_gridSize) & (newY >= 0) & (newY < m_gridSize);
				int currLoc = newX + newY * m_gridSize;
				moveStates[i][j + 1] = (currLoc) * (isValid)+NVALID_MOVE * (!isValid);

				// calculate the nearest location for self
				int currDist = Distance(currLoc, state[0], m_gridSize);
				if (currDist < minDist & isValid & currLoc != state[0])
				{
					minDist = currDist;
					minIdx = j;
				}
			}

			//assumption: not support moving toward of non involved
			if (IsEnemy(i + 1) && (minIdx >= 0 & m_enemyVec[i].GetMovement().GetToward() > 0.0))
			{
				moveStates[i][s_NUM_DIRECTIONS + 1] = x + s_lutDirections[minIdx][0] + (y + s_lutDirections[minIdx][1]) * m_gridSize;
			}
			else
			{
				moveStates[i][s_NUM_DIRECTIONS + 1] = NVALID_MOVE;
			}
		}
		else
		{
			for (int j = 1; j < s_NUM_OPTIONS_MOVE; ++j)
				moveStates[i][j] = NVALID_MOVE;
		}
	}
}

void POMDP_Writer::CalcObs(std::string& buffer)
{
	intVec state(CountMovableObj());
	CalcObsRec(state, 0, buffer);

	buffer += "\nO: * : " + s_WinState + " : " "oWin 1.0";
	buffer += "\nO: * : " + s_LossState + " : " "oLoss 1.0";
}
void POMDP_Writer::CalcObsRec(intVec& state, int currIdx, std::string & buffer)
{
	if (currIdx == state.size())
	{
		// arriving here when state is initialize to a state. run on this state calculation of observations
		CalcObsSingleState(state, buffer);
		buffer += "\n";
	}
	else
	{
		// run on all possible locations. if there location is valid call for the calculation of the next object
		for (int i = 0; i < m_gridSize*m_gridSize; ++i)
		{
			state[currIdx] = i;
			if (NoRepeats(state, currIdx))
			{
				CalcObsRec(state, currIdx + 1, buffer);
			}
		}

		if (IsEnemy(currIdx))
		{
			state[currIdx] = m_gridSize * m_gridSize;
			CalcObsRec(state, currIdx + 1, buffer);
		}
	}

}

void POMDP_Writer::CalcObsSingleState(intVec& state, std::string& buffer)
{
	std::string prefix = "O: * : " + GetStringState(state) + " : o";
	std::vector<bool> inRange(state.size());
	intVec newState(state);
	mapProb pMap;

	// create a vector indicating which one of the different object is in range
	for (int i = 1; i < state.size(); ++i)
	{
		if (InObsRange(state[0], state[i], m_gridSize, m_self.GetRangeObs()))
		{
			inRange[i] = true;
		}
		else
		{
			inRange[i] = false;
		}
	}

	CalcObsMapRec(newState, state, pMap, inRange, 1.0, 1);

	// add observations to buffer
	int numStates = m_gridSize * m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, numStates](pairMap itr)
	{	buffer += prefix;	POMDP_Writer::AddStateToBuffer(buffer, itr, numStates); });
}

void POMDP_Writer::CalcObsMapRec(intVec& state, intVec& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, int currIdx)
{
	// stopping condition: arriving to the end of the state vec
	if (currIdx == state.size())
	{
		// insert p to map
		if (NoRepeatsAll(state))
			pMap[state] += pCurr;
		else
			pMap[originalState] += pCurr;
	}
	else
	{
		// if curr object is dead continue to the next object
		if (state[currIdx] == m_gridSize * m_gridSize)
		{
			CalcObsMapRec(state, originalState, pMap, inRange, pCurr, currIdx + 1);
		}
		// if the original location is in range & the current location is the original location and there are no repetition the location is observable
		else if (inRange[currIdx] & (state[currIdx] == originalState[currIdx]) & NoRepeats(state, currIdx))
		{
			CalcObsMapRec(state, originalState, pMap, inRange, pCurr * m_self.GetPObs(), currIdx + 1);
			DivergeObs(state, originalState, pMap, inRange, pCurr * (1 - m_self.GetPObs()), currIdx, true);
		}
		else
		{
			// TODO : still not implemented what happen when obs range is smaller than grid 
			DivergeObs(state, originalState, pMap, inRange, pCurr, currIdx, false);
		}
	}

}

void POMDP_Writer::DivergeObs(intVec& state, intVec& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, int currIdx, bool avoidCurrLoc)
{
	// if the enemy is dead do not run on other options(because they are not possible)
	if (state[currIdx] == m_gridSize * m_gridSize)
	{
		CalcObsMapRec(state, originalState, pMap, inRange, pCurr, currIdx + 1);
		return;
	}

	int currLocation = state[currIdx];
	//calculate how many diversion there will be decrease repetitions(- currIdx), add not important repetition(DEAD_ENEMY) and decrease the current location if necessary
	int pDivision = 8;

	int currLoc = state[currIdx];
	for (int i = 0; i < 8; ++i)
	{
		if (InBoundary(currLoc, s_lutDirections[i][0], s_lutDirections[i][1], m_gridSize))
			state[currIdx] = currLoc + s_lutDirections[i][0] + s_lutDirections[i][1] * m_gridSize;
		else
			state[currIdx] = currLoc;
		
		CalcObsMapRec(state, originalState, pMap, inRange, pCurr / pDivision, currIdx + 1);
	}

	state[currIdx] = currLocation;
}

bool POMDP_Writer::InObsRange(int self, int object, int gridSize, int range)
{
	if (object == gridSize * gridSize)
	{
		return true;
	}
	int xSelf = self % gridSize;
	int ySelf = self / gridSize;
	int xObj = object % gridSize;
	int yObj = object / gridSize;

	if (Abs(xSelf - xObj) <= range && Abs(ySelf - yObj) <= range)
	{
		return true;
	}
	return false;
}


inline bool POMDP_Writer::IsEnemy(int idx) const
{
	return idx - 1 < m_enemyVec.size();
}

inline int POMDP_Writer::CountMovableObj() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

int POMDP_Writer::CountEnemies() const
{
	return m_enemyVec.size();
}

int POMDP_Writer::CountNInv() const
{
	return m_nonInvolvedVec.size();
}

int POMDP_Writer::CountShelters() const
{
	return m_shelterVec.size();
}

int POMDP_Writer::GetGridSize() const
{
	return m_gridSize;
}

inline bool POMDP_Writer::AnyDead(intVec& state) const
{
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		if (state[i + 1] == m_gridSize * m_gridSize)
		{
			return true;
		}
	}
	return false;
}


std::ostream& operator<<(std::ostream& o, const POMDP_Writer& pomdp)
{
	o << "\ngridSize : " << pomdp.m_gridSize <<
		"\nnumber of enemies : " << pomdp.m_enemyVec.size() <<
		"\nnumber of non involved : " << pomdp.m_nonInvolvedVec.size() <<
		"\nnumber of shelters : " << pomdp.m_shelterVec.size() << "\n";
	return o;
}

std::ofstream & operator<<(std::ofstream & out, const POMDP_Writer & pomdp)
{
	out.write(reinterpret_cast<const char *>(&pomdp.m_gridSize), sizeof(int));
	out.write(reinterpret_cast<const char *>(&pomdp.m_targetIdx), sizeof(int));
	out.write(reinterpret_cast<const char *>(&pomdp.m_discount), sizeof(int));

	out << pomdp.m_self;
	
	int vecSize = pomdp.m_enemyVec.size();
	out.write(reinterpret_cast<const char *>(&vecSize), sizeof(int));
	for (auto v : pomdp.m_enemyVec)
	{
		out << v;
	}

	vecSize = pomdp.m_nonInvolvedVec.size();
	out.write(reinterpret_cast<const char *>(&vecSize), sizeof(int));
	for (auto v : pomdp.m_nonInvolvedVec)
	{
		out << v;
	}

	vecSize = pomdp.m_shelterVec.size();
	out.write(reinterpret_cast<const char *>(&vecSize), sizeof(int));
	for (auto v : pomdp.m_shelterVec)
	{
		out << v;
	}

	return out;
}

std::ifstream & operator>>(std::ifstream & in, POMDP_Writer & pomdp)
{
	READ(in, reinterpret_cast<char *>(&pomdp.m_gridSize), sizeof(int));
	READ(in, reinterpret_cast<char *>(&pomdp.m_targetIdx), sizeof(int));
	READ(in, reinterpret_cast<char *>(&pomdp.m_discount), sizeof(int));

	in >> pomdp.m_self;

	int size;
	READ(in, reinterpret_cast<char *>(&size), sizeof(int));
	pomdp.m_enemyVec.resize(size);
	for (auto v : pomdp.m_enemyVec)
	{
		in >> v;
	}

	in.read(reinterpret_cast<char *>(&size), sizeof(int));
	pomdp.m_nonInvolvedVec.resize(size);
	for (auto v : pomdp.m_nonInvolvedVec)
	{
		in >> v;
	}

	in.read(reinterpret_cast<char *>(&size), sizeof(int));
	pomdp.m_shelterVec.resize(size);
	for (auto v : pomdp.m_shelterVec)
	{
		in >> v;
	}

	return in;
}
