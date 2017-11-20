#include "nxnGridOffline.h"

#include <iostream>		// cout
#include <string>		// string
#include <algorithm>	// algorithms
#include <fstream>      // std::ofstream
#include <sstream>		// ostringstream
#include <iomanip>		// set_percision


inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;

	return xDiff * xDiff + yDiff * yDiff;
}

static const std::string s_WinState = "Win";
static const std::string s_LossState = "Loss";

/// value in move states for non-valid move
static const int NVALID_MOVE = -1;
/// all directions which are also all possible moves
static const int s_NUM_DIRECTIONS = 8;
/// lut for all direction changes
static int s_lutDirections[s_NUM_DIRECTIONS][2] = { { 0,1 },{ 0,-1 },{ 1,0 },{ -1,0 },{ 1,1 },{ 1,-1 },{ -1,1 },{ -1,-1 } };

/// second square changes surrounding the point
static const int s_numPixelsSecondSquare = 16;
static const int s_lutSecondSquare[s_numPixelsSecondSquare][2] = { { 0, 2 }, { 0, -2 }, { 2, 0 }, { -2, 0 },
																	{ 1, 2 }, { 1, -2 }, { 2, 1 }, { -2, 1 },
																	{ -1, 2 }, { -1, -2 }, { 2, -1 }, { -2, -1 },
																	{ 2, 2 }, { 2, -2 }, { -2, 2 }, { -2, -2 } };

/// number of moves of an object from a specific location (stay, num moves, moving toward robot)
static const int s_NUM_OPTIONS_MOVE = s_NUM_DIRECTIONS + 2;

double nxnGridOffline::s_pLeftProbability = 1.0;

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

static double CalcPAUB(std::vector<double> & pVec)
{
	if (pVec.size() == 0)
		return 0.0;
	if (pVec.size() == 1)
		return pVec[0];
	else if (pVec.size() == 2)
		return pVec[0] + pVec[1] - pVec[0] * pVec[1];
	else if (pVec.size() == 3)
		return pVec[0] + pVec[1] + pVec[2] - pVec[0] * pVec[1] - pVec[1] * pVec[2] - pVec[0] * pVec[2] + pVec[0] * pVec[1] * pVec[2];
	// need to continue if i want to allow more than 3 enemies
}

nxnGridOffline::nxnGridOffline(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount)
	: m_gridSize(gridSize)
	, m_targetIdx(targetIdx)
	, m_discount(discount)
	, m_isFullyObs(isFullyObs)
	, m_self(self)
	, m_enemyVec()
	, m_nonInvolvedVec()
	, m_shelterVec()
{
}


void nxnGridOffline::UpdateSelf(Self_Obj && obj)
{
	m_self = obj;
}

void nxnGridOffline::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
}

void nxnGridOffline::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));
}

void nxnGridOffline::AddObj(ObjInGrid&& obj)
{
	m_shelterVec.emplace_back(std::forward<ObjInGrid>(obj));
}

void nxnGridOffline::SetLocationSelf(Coordinate & newLocation)
{
	m_self.SetLocation( newLocation );
}

void nxnGridOffline::SetLocationEnemy(Coordinate & newLocation, int idxObj)
{

	m_enemyVec[idxObj - 1].SetLocation(newLocation);
}

void nxnGridOffline::SetLocationNonInv(Coordinate & newLocation, int idxObj)
{
	m_nonInvolvedVec[idxObj - 1 - m_enemyVec.size()].SetLocation(newLocation);
}

void nxnGridOffline::SetLocationShelter(Coordinate & newLocation, int idxObj)
{
	m_shelterVec[idxObj - 1 - m_enemyVec.size() - m_nonInvolvedVec.size()].SetLocation(newLocation);
}

void nxnGridOffline::SetTarget(int idx)
{
	m_targetIdx = idx;
}

void nxnGridOffline::SetGridSize(int gridSize)
{
	m_gridSize = gridSize;
}

long nxnGridOffline::StateCount2StateIdx(long stateCount) const
{
	intVec state(CountMovableObj());
	// add moving objects to state
	long add1ToCount = stateCount + 1;
	FindStateCount(state, add1ToCount, 0);
	
	// insert shelter location to state
	if (m_shelterVec.size() > 0)
	{
		Coordinate s = m_shelterVec[0].GetLocation();
		state.emplace_back(s.GetIdx(m_gridSize));
	}

	return State2Idx(state, m_gridSize);
}

void nxnGridOffline::FindStateCount(intVec & state, long & count, int currIdx) const
{
	// stopping condition when finish running on all objects
	if (state.size() == currIdx)
	{
		--count;
	}
	else
	{
		// run on all possible states (without repetitions)
		for (int i = 0; i < m_gridSize * m_gridSize & count != 0; ++i)
		{
			state[currIdx] = i;
			if (NoRepeats(state, currIdx))
			{
				FindStateCount(state, count, currIdx + 1);
			}
		}

		if (IsEnemy(currIdx) & count != 0)
		{
			state[currIdx] = m_gridSize * m_gridSize;
			FindStateCount(state, count, currIdx + 1);
		}
	}
}

long nxnGridOffline::State2Idx(intVec & state, int gridSize)
{
	long idx = 0;
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

nxnGridOffline::intVec nxnGridOffline::Idx2State(long stateIdx)
{
	intVec state(CountMovableObj());
	int numStates = m_gridSize * m_gridSize + 1;

	// don't treat shelter location
	stateIdx /= numStates;
	// running on all varied objects and concluding from the obs num the observed state
	for (int i = CountMovableObj() - 1; i >= 0; --i)
	{
		state[i] = stateIdx % numStates;
		stateIdx /= numStates;
	}

	return state;
}


void nxnGridOffline::ObservationsAndRewards(FILE * fptr)
{
	std::string buffer;
	// calculate observations
	CalcObs(buffer);
	// add rewards
	buffer += "\n\nR: * : * : * : * 0.0";
	buffer += "\nR: * : * : " + s_WinState + " : * 50";
	buffer += "\nR: * : * : " + s_LossState + " : * -100\n";
	buffer += "\nR: * : " + s_WinState + " : " + s_WinState + " : * 0.0";
	buffer += "\nR: * : " + s_LossState + " : " + s_LossState + " : * 0.0";
	// save to file 
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; return; }
}

void nxnGridOffline::CalcStatesAndObs(const char * type, std::string& buffer)
{
	intVec state(CountMovableObj());
	CalcS_ORec(state, 0, type, buffer);
}

void nxnGridOffline::CalcS_ORec(intVec& state, int currIdx, const char * type, std::string & buffer)
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

		// add non_observed location for observation (state[non observed] = self location)
		if (*type == 'o' & currIdx > 0)
		{
			state[currIdx] = state[0];
			CalcS_ORec(state, currIdx + 1, type, buffer);
		}
	}
}

bool nxnGridOffline::NoRepetitionCheckAndCorrect(intVec& state, std::vector<intVec> & moveStates, intVec & arrOfIdx) const
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

bool nxnGridOffline::NoRepeatsLocation(intVec& state, int loc) const
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

bool nxnGridOffline::NoRepeatsAll(intVec& state) const
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

bool nxnGridOffline::NoRepeats(intVec& state, int idx) const
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

void nxnGridOffline::CalcStartState(std::string& buffer)
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

void nxnGridOffline::CalcStartStateRec(intVec& state, intVec& initState, int currIdx, std::string & buffer)
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

void nxnGridOffline::AddTargetPositionRec(intVec & state, int currIdx, std::string & buffer)
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

void nxnGridOffline::CreateShleterVec(intVec & shelters) const
{
	for (auto v : m_shelterVec)
		shelters.push_back(v.GetLocation().GetIdx(m_gridSize));
}

bool nxnGridOffline::IsNonInvDead(intVec & state) const
{
	bool isDead = false;

	// searching for dead non-involved
	for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
		isDead |= (state[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize);

	return isDead;
}

int nxnGridOffline::FindNearestShelter(int location) const
{
	// assumption : support only 1 shelter
	return m_shelterVec[0].GetLocation().GetIdx(m_gridSize);
}

double nxnGridOffline::PositionSingleState(intVec & newState, intVec & currentState, intVec & shelters, std::string & action, std::string & buffer) const
{
	std::string prefix = "T: " + action + " : " + GetStringState(currentState) + " : ";

	// if the enemy is not dead calculate his attack (not support more than 1 enemy so far)
	std::vector<double> individualProb2Kill;
	for (int e = 0; e < m_enemyVec.size(); ++e)
	{
		if (currentState[e + 1] != m_gridSize * m_gridSize)
		{
			Attack::shootOutcomes result;
			m_enemyVec[e].GetAttack()->AttackOffline(currentState[e + 1], currentState[0], currentState, shelters, m_gridSize, result);
			for (auto v : result)
			{
				if (v.first[0] == m_gridSize * m_gridSize)
					individualProb2Kill.emplace_back(v.second);
			}
		}
	}
	double pToDead = CalcPAUB(individualProb2Kill);
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
	std::vector<int> arrOfIdx(CountMovableObj() - 1, 0);

	mapProb pMap;
	// calculate the probability of each move state and insert it to pMap
	AddMoveStatesRec(newState, moveStates, arrOfIdx, 0, pMap);
	
	// insert the move states to the buffer
	int numStates = m_gridSize * m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, numStates](pairMap itr)
	{	buffer += prefix;	AddStateToBuffer(buffer, itr, numStates); });

	return pToDead;
}

void nxnGridOffline::AddStateToBuffer(std::string& buffer, pairMap & itr, int numLocations)
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
	buffer += " " + Dbl2Str(itr.second) + "\n";
}

std::string nxnGridOffline::GetStringState(intVec & state) const
{
	return GetStringState(state, "s");
}

std::string nxnGridOffline::GetStringState(intVec & state, const char *type) const
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

bool nxnGridOffline::SearchForShelter(int location) const
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

bool nxnGridOffline::InBoundary(int state, int advanceFactor, int gridSize)
{
	int x = state % gridSize;
	int y = state / gridSize;
	int xdiff = advanceFactor % gridSize;
	int ydiff = advanceFactor / gridSize;

	return ((x + xdiff) >= 0) & ((x + xdiff) < gridSize) & ((y + ydiff) >= 0) & ((y + ydiff) < gridSize);
}

bool nxnGridOffline::InBoundary(int location, int xChange, int yChange, int gridSize)
{
	int x = location % gridSize + xChange;
	int y = location / gridSize + yChange;
	// return true if the location is in boundary
	return x >= 0 & x < gridSize & y >= 0 & y < gridSize;
}

void nxnGridOffline::AddMoveStatesRec(intVec & state, std::vector<intVec> & moveStates, intVec & arrOfIdx, int currIdx, mapProb & pMap) const
{
	if (currIdx == state.size() - 1)
	{
		double p = CalcProb2Move(arrOfIdx);
		pMap[MoveToIdx(state, moveStates, arrOfIdx)] += p;
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


nxnGridOffline::intVec nxnGridOffline::MoveToIdx(intVec state, std::vector<intVec> & moveStates, intVec arrOfIdx) const
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

double nxnGridOffline::CalcProb2Move(const intVec & arrOfIdx) const
{
	// insert to p the whole (replacement of 1 due to cases that reduce probability)
	double pMoveState = s_pLeftProbability;
	
	// multiply with the p(enemy = moveState)
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		int relativeIdx = arrOfIdx[i];

		pMoveState *= m_enemyVec[i].GetMovement().GetStay() * (relativeIdx == 0) +
			m_enemyVec[i].GetMovement().GetEqual() * (relativeIdx > 0 & relativeIdx < s_NUM_OPTIONS_MOVE - 1) +
			m_enemyVec[i].GetMovement().GetToward() * (relativeIdx == s_NUM_OPTIONS_MOVE - 1);
	}
	// multiply with the p(non-involved = moveState)
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		int relativeIdx = arrOfIdx[i + m_enemyVec.size()];
		pMoveState *= m_nonInvolvedVec[i].GetMovement().GetStay() * (relativeIdx == 0) +
			m_nonInvolvedVec[i].GetMovement().GetEqual() * (relativeIdx > 0 & relativeIdx < s_NUM_OPTIONS_MOVE - 1) +
			m_nonInvolvedVec[i].GetMovement().GetToward() * (relativeIdx == s_NUM_OPTIONS_MOVE - 1);
	}

	return pMoveState;
}

void nxnGridOffline::CalcMoveStates(intVec & state, std::vector<intVec> & moveStates) const
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

void nxnGridOffline::CalcObs(std::string& buffer)
{
	intVec state(CountMovableObj());
	CalcObsRec(state, 0, buffer);

	buffer += "\nO: * : " + s_WinState + " : " "oWin 1.0";
	buffer += "\nO: * : " + s_LossState + " : " "oLoss 1.0";
}
void nxnGridOffline::CalcObsRec(intVec& state, int currIdx, std::string & buffer)
{
	if (currIdx == state.size())
	{
		// arriving here when state is initialize to a state. run on this state calculation of observations
		if (m_isFullyObs)
			buffer += "O: * : " + GetStringState(state) + " : " + GetStringState(state, "o") + " 1\n";
		else
			CalcObsSingleState(state, buffer);

		buffer += "\n";
	}
	else
	{
		// run on all possible locations. if there location is valid call for the calculation of the next object
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
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

void nxnGridOffline::CalcObsSingleState(intVec& state, std::string& buffer)
{
	std::string prefix = "O: * : " + GetStringState(state) + " : o";
	
	intVec newState(state);
	mapProb pMap;

	CalcObsMapRec(newState, state, pMap, 1.0, 1);

	// add observations to buffer
	int numStates = m_gridSize * m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, numStates](pairMap itr)
	{	buffer += prefix;	nxnGridOffline::AddStateToBuffer(buffer, itr, numStates); });
}

void nxnGridOffline::CalcObsMapRec(intVec& observedState, intVec& state, mapProb& pMap, double pCurr, int currObj)
{
	// stopping condition: arriving to the end of the state vec
	if (currObj == state.size())
	{
		// insert p to map
		pMap[observedState] += pCurr;
	}
	else
	{
		intVec observableLocations;
		m_self.GetObservation()->InitObsAvailableLocations(state[0], state[currObj], state, m_gridSize, observableLocations);
		for (auto obsLoc : observableLocations)
		{
			observedState[currObj] = obsLoc;
			double pObs = m_self.GetObservation()->GetProbObservation(state[0], state[currObj], m_gridSize, obsLoc);
			CalcObsMapRec(observedState, state, pMap, pCurr * pObs, currObj + 1);
		}
	}
}

inline bool nxnGridOffline::IsEnemy(int idx) const
{
	return idx - 1 < m_enemyVec.size();
}

inline int nxnGridOffline::CountMovableObj() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

int nxnGridOffline::CountEnemies() const
{
	return m_enemyVec.size();
}

int nxnGridOffline::CountNInv() const
{
	return m_nonInvolvedVec.size();
}

int nxnGridOffline::CountShelters() const
{
	return m_shelterVec.size();
}

int nxnGridOffline::GetGridSize() const
{
	return m_gridSize;
}

inline bool nxnGridOffline::AnyDead(intVec& state) const
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

// translate to string with higher precision
std::string nxnGridOffline::Dbl2Str(double d)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(10) << d;              //convert double to string w fixed notation, hi precision
	std::string s = ss.str();                                    //output to std::string
	s.erase(s.find_last_not_of('0') + 1, std::string::npos);     //remove trailing 000s    (123.1200 => 123.12,  123.000 => 123.)

	double a = strtod(s.c_str(), NULL);
	return (s[s.size() - 1] == '.') ? s.substr(0, s.size() - 1) : s; //remove dangling decimal (123. => 123)
}


std::ostream& operator<<(std::ostream& o, const nxnGridOffline& pomdp)
{
	o << "\ngridSize : " << pomdp.m_gridSize <<
		"\nnumber of enemies : " << pomdp.m_enemyVec.size() <<
		"\nnumber of non involved : " << pomdp.m_nonInvolvedVec.size() <<
		"\nnumber of shelters : " << pomdp.m_shelterVec.size() << "\n";
	return o;
}
