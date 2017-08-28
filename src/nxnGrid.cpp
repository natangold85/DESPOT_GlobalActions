#include <string>
#include <math.h>

#include "..\include\despot\solver\pomcp.h"
#include "nxnGrid.h"
#include "Coordinate.h"

namespace despot 
{

/// string array for all actions (enum as idx)
static std::vector<std::string> ACTION_STR(nxnGrid::NUM_ACTIONS);

/// changes surrounding a point
static const int s_numMoves = 8;
static const int s_lutDirections[s_numMoves][2] = { { 0,1 },{ 0,-1 },{ 1,0 },{ -1,0 },{ 1,1 },{ 1,-1 },{ -1,1 },{ -1,-1 } };


// init static members

int nxnGridState::s_sizeState = 1;
int nxnGridState::s_endEnemyIdx = 1;

int nxnGridState::s_gridSize = 0;
int nxnGridState::s_targetIdx = 0;

std::vector<ObjInGrid> nxnGridState::s_shelters;

const double nxnGrid::REWARD_WIN = 50.0;
const double nxnGrid::REWARD_LOSS = -100.0;
const double nxnGrid::REWARD_KILL_ENEMY = 0.0;
const double nxnGrid::REWARD_KILL_NINV = REWARD_LOSS;

// for synchronizing between sarsop rewards to despot rewards
static double REWARD_WIN_MAP = 1.0; 
static double REWARD_LOSS_MAP = -2.0;
enum nxnGrid::CALCULATION_TYPE nxnGrid::s_calculationType = WITHOUT;

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
* nxnGridState Functions
* =============================================================================*/

nxnGridState::nxnGridState(int state_id, double weight)
	: State(state_id, weight)
{
}

std::string nxnGridState::text() const
{
	intVec state = IdxToState(state_id);
	std::string ret = "\n";
	for (int y = 0; y < s_gridSize; ++y)
	{
		for (int x = 0; x < s_gridSize; ++x)
		{
			int loc = x + y * s_gridSize;
			// print object or target at location
			if (state[0] == loc)
				ret += "M";
			else if (EnemyLoc(state, loc))
				ret += "E";
			else if (NInvLoc(state, loc))
				ret += "N";
			else if (loc == s_targetIdx)
				ret += "T";
			else if (ShelterLoc(loc))
				ret += "S";
			else
				ret += "_";
		}
		ret += "\n";
	}

	return ret;
}

void nxnGridState::UpdateState(int newStateIdx)
{
	state_id = newStateIdx;
}

void nxnGridState::UpdateState(intVec newState)
{
	state_id = StateToIdx(newState);
}

intVec nxnGridState::IdxToState(int idx)
{
	intVec state(s_sizeState);
	int numStates = s_gridSize * s_gridSize + 1;

	// running on all varied objects and concluding from the obs num the observed state
	for (int i = s_sizeState - 1; i >= 0; --i)
	{
		state[i] = idx % numStates;
		idx /= numStates;
	}

	return state;
}

int nxnGridState::StateToIdx(intVec &state)
{
	int idx = 0;
	// add 1 for num states for dead objects
	int numStates = s_gridSize * s_gridSize + 1;
	// idx = s[0] * numstates^n + s[1] * numstates^(n - 1) ...  + s[n] * numstates^(0)
	for (int i = 0; i < state.size(); ++i)
	{
		idx *= numStates;
		idx += state[i];
	}

	return idx;
}

int nxnGridState::MaxState()
{
	int numStates = s_gridSize * s_gridSize + 1;

	return pow(numStates, s_sizeState);
}

bool nxnGridState::EnemyLoc(intVec & state, int location) const
{
	for (int i = 1; i < s_endEnemyIdx; ++i)
	{
		if (state[i] == location)
			return true;
	}
	return false;
}

bool nxnGridState::NInvLoc(intVec & state, int location) const
{
	for (int i = s_endEnemyIdx; i < state.size(); ++i)
	{
		if (state[i] == location)
			return true;
	}
	return false;
}

bool nxnGridState::ShelterLoc(int location) const
{
	for (auto v : s_shelters)
	{
		if (location == v.GetLocation().GetIdx(s_gridSize))
			return true;
	}

	return false;
}

/* =============================================================================
* nxnGrid Functions
* =============================================================================*/

nxnGrid::nxnGrid(int gridSize, int target, Self_Obj & self, std::shared_ptr<StateActionLUT> lut, enum CALCULATION_TYPE calcType)
	: m_gridSize(gridSize)
	, m_targetIdx(target)
	, m_self(self)
	, m_enemyVec()
	, m_shelters()
	, m_nonInvolvedVec()
	, m_LUT(lut)
{
	// init size  of state for nxnGridstate
	nxnGridState::s_sizeState = 1;
	nxnGridState::s_gridSize = gridSize;
	nxnGridState::s_targetIdx = target;

	// init vector for string of actions
	ACTION_STR[MOVE_TO_TARGET] = "Move To Target";
	ACTION_STR[MOVE_TO_SHELTER] = "Move To Shelter";
	ACTION_STR[ATTACK] = "Attack";
	ACTION_STR[MOVE_FROM_ENEMY] = "Move From Enemy";

	s_calculationType = calcType;
}

void nxnGrid::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
	++nxnGridState::s_sizeState;
	++nxnGridState::s_endEnemyIdx;
}

void nxnGrid::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));
	++nxnGridState::s_sizeState;
}

void nxnGrid::AddObj(ObjInGrid&& obj)
{
	m_shelters.emplace_back(std::forward<ObjInGrid>(obj));
	nxnGridState::s_shelters.emplace_back(std::forward<ObjInGrid>(obj));
}



int nxnGrid::GetObsLoc(OBS_TYPE obs, int objIdx) const
{
	const intVec obsState = nxnGridState::IdxToState(obs);

	return obsState[objIdx];
}

bool nxnGrid::InObsRange(OBS_TYPE obs, int objIdx) const
{
	const intVec obsState = nxnGridState::IdxToState(obs);

	return InRange(obsState[0], obsState[objIdx], m_self.GetRangeObs(), m_gridSize);
}

bool nxnGrid::Step(State& s, double randomSelfAction, int a, double& reward, OBS_TYPE& obs) const
{
	nxnGridState& stateClass = static_cast<nxnGridState&>(s);
	intVec state = stateClass.IdxToState(stateClass);
	enum ACTION action = static_cast<enum ACTION>(a);

	// drawing more random numbers for each variable
	double randomSelfObservation = rand();
	randomSelfObservation /= RAND_MAX;

	std::vector<double> randomObjectMoves = CreateRandomVec(CountMovingObjects() - 1);
	std::vector<double> randomEnemiesAttacks = CreateRandomVec(m_enemyVec.size());

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
	else if(action == MOVE_TO_SHELTER)
	{
		// change location of robot if possible according to action
		MoveToShelter(state, randomSelfAction);
	}
	else if (action == ATTACK)
	{
		if (state[1] != m_gridSize * m_gridSize)
		{
			Attack(state, randomSelfAction);
			for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
			{
				if (state[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize)
				{
					reward = REWARD_KILL_NINV;
					return true;
				}
			}
			reward = REWARD_KILL_ENEMY * (state[1] == m_gridSize * m_gridSize);
		}
	}
	else // action = movefromenemy
	{
		MoveFromEnemy(state, randomSelfAction);
	}

	// set next position of the objects on grid
	SetNextPosition(state, randomObjectMoves);
	// update observation
	obs = FindObservation(state, randomSelfObservation);
	//update state
	stateClass.UpdateState(state);
	return false;
}

int nxnGrid::NumStates() const
{
	int numStateForObj = m_gridSize * m_gridSize;

	return pow(numStateForObj, CountMovingObjects());
}

double nxnGrid::ObsProb(OBS_TYPE obs, const State & s, int action) const
{
	const nxnGridState& stateIdx = static_cast<const nxnGridState&>(s);
	intVec state = nxnGridState::IdxToState(stateIdx);

	const intVec obsState = nxnGridState::IdxToState(obs);
	// if observation is not including the location of the robot return 0
	if (state[0] != obsState[0])
	{
		return 0.0;
	}

	double pObs = 1.0;
	double pNear = (1.0 - m_self.GetPObs()) / 8;

	// run on all non-self objects location
	for (int i = 1; i < CountMovingObjects(); ++i)
	{
		// if the object is dead we have 1.0 prob to know it, if not decrease num possible locations by 1
		if (state[i] == m_gridSize * m_gridSize)
			continue;

		// if the location is in range of the obs_range of the robot
		if (InRange(state[0], state[i], m_self.GetRangeObs(), m_gridSize))
		{
			//if the location of the observation is equal to the location of the object multiply p by p(ToObserve)
			if (state[i] == obsState[i])
			{
				pObs *= m_self.GetPObs();
			}
			else if (IsNear(state[i], obsState[i], m_gridSize))	//else if the location is near the real place multiply by pNear
			{
				pObs *= pNear;
			}
			else // if the observed location is not near the real location pObs = 0.0
				return 0.0;
			
		}
	}
	
	return pObs;
}

State * nxnGrid::CreateStartState(std::string type) const
{
	intVec state(CountMovingObjects());

	state[0] = m_self.GetLocation().GetIdx(m_gridSize);
	
	for (int i = 0; i < m_enemyVec.size(); ++i)
		state[i + 1] = m_enemyVec[i].GetLocation().GetIdx(m_gridSize);

	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
		state[i + 1 + m_enemyVec.size()] = m_nonInvolvedVec[i].GetLocation().GetIdx(m_gridSize);


	return new nxnGridState(nxnGridState::StateToIdx(state));
}

Belief * nxnGrid::InitialBelief(const State * start, std::string type) const
{
	std::vector<State*> particles;
	intVec state(CountMovingObjects());

	// create belief states given self location
	state[0] = m_self.GetLocation().GetIdx(m_gridSize);
	InsertParticlesRec(state, particles, 1.0, 1);

	return new ParticleBelief(particles, this);
}

State * nxnGrid::Allocate(int state_id, double weight) const
{
	nxnGridState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State * nxnGrid::Copy(const State * particle) const
{
	nxnGridState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const nxnGridState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void nxnGrid::Free(State * particle) const
{
	memory_pool_.Free(static_cast<nxnGridState*>(particle));
}

int nxnGrid::NumActiveParticles() const
{
	return memory_pool_.num_allocated();
}

void nxnGrid::PrintState(const State & s, std::ostream & out) const
{
	const nxnGridState& state = static_cast<const nxnGridState&>(s);
	out << state.text() << std::endl;
}

void nxnGrid::PrintBelief(const Belief & belief, std::ostream & out) const
{
}

void nxnGrid::PrintObs(const State & state, OBS_TYPE obs, std::ostream & out) const
{
	intVec obsState = nxnGridState::IdxToState(obs);
	out << "O: ";
	for (auto v : obsState)
	{
		out << v << ",";
	}
	out << "\n";
}

void nxnGrid::PrintAction(int action, std::ostream & out) const
{
	out << ACTION_STR[action] << std::endl;
}

void nxnGrid::MoveToTarget(intVec & state, double random) const
{
	Coordinate target(m_targetIdx % m_gridSize, m_targetIdx / m_gridSize);
	MoveToLocation(state, target, random);
}

void nxnGrid::MoveToShelter(intVec & state, double random) const
{
	int shelterLoc = m_shelters[0].GetLocation().GetIdx(m_gridSize);
	
	if (shelterLoc != state[0])
	{
		Coordinate shelter(shelterLoc % m_gridSize, shelterLoc / m_gridSize);
		MoveToLocation(state, shelter, random);
	}
}

void nxnGrid::Attack(intVec & state, double random) const
{
	// if the enemy is already dead do nothing
	if (state[1] == m_gridSize * m_gridSize)
		return;

	if (InRange(state[0], state[1], m_self.GetRange(), m_gridSize))
	{
		intVec shelters(m_shelters.size());
		for (size_t i = 0; i < m_shelters.size(); ++i)
			shelters[i] = m_shelters[i].GetLocation().GetIdx(m_gridSize);

		m_self.CalcSelfAttackDESPOT(state, shelters, m_gridSize, random);
	}
	else
	{
		Coordinate enemy(state[1] % m_gridSize, state[1] / m_gridSize);
		MoveToLocation(state, enemy, random);
	}
}

void nxnGrid::MoveFromEnemy(intVec & state, double random) const
{
	// if according to probability the robot is moving and the enemy is not dead move toward enemy
	if (random < m_self.GetSelfPMove() & state[1] != m_gridSize * m_gridSize)
	{
		Coordinate enemy(state[1] % m_gridSize, state[1] / m_gridSize);
		state[0] = MoveFromLocation(state, enemy);
	}
}

void nxnGrid::MoveToLocation(intVec & state, Coordinate & goTo, double random) const
{
	int move = MoveToLocationIMP(state, goTo);

	random -= m_self.GetMovement().GetToward();
	if (random <= 0)
		state[0] = move;	
}

int nxnGrid::MoveFromLocation(intVec & state, Coordinate & goFrom) const
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


int nxnGrid::MoveToLocationIMP(intVec & state, Coordinate & goTo) const
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


bool nxnGrid::InRange(int locationSelf, int locationObj, double range, int gridSize)
{
	Coordinate self(locationSelf % gridSize, locationSelf / gridSize);
	Coordinate enemy(locationObj % gridSize, locationObj / gridSize);

	// return true when distance is <= from range
	return self.RealDistance(enemy) <= range;
}

bool nxnGrid::CalcIfDead(int enemyIdx, intVec state, double & randomNum) const
{
	intVec shelters(m_shelters.size());

	for (size_t i = 0; i < shelters.size(); ++i)
		shelters[i] = m_shelters[i].GetLocation().GetIdx(m_gridSize);

	return m_enemyVec[enemyIdx].CalcEnemyAttackDESPOT(state[0], state[enemyIdx + 1], state, shelters, m_gridSize, randomNum);
}


void nxnGrid::MoveObjectLocation(const intVec & beliefState, intVec & scaledState, int objIdx, int gridSize) const
{
	Coordinate objectRealLoc(beliefState[objIdx] % m_gridSize, beliefState[objIdx] / m_gridSize);
	Coordinate objectLoc(scaledState[objIdx] % gridSize, scaledState[objIdx] / gridSize);
	
	int minDist = 100000;
	int bestLocation = -1;
	double scaleBackward = static_cast<double>(gridSize) / m_gridSize;

	// run on all possible moves of enemy and calculate which valid location is closest to object
	for (size_t i = 0; i < s_numMoves; ++i)
	{
		Coordinate newLoc(s_lutDirections[i][0], s_lutDirections[i][1]);
		newLoc += objectLoc;
		if (ValidLegalLocation(scaledState, newLoc, objIdx, gridSize))
		{
			newLoc /= scaleBackward;
			int dist = newLoc.Distance(objectRealLoc);
			if (dist < minDist)
			{
				minDist = dist;
				bestLocation = i;
			}
		}
	}

	scaledState[objIdx] += s_lutDirections[bestLocation][0] + s_lutDirections[bestLocation][1] * gridSize;

	int shelterIdx = 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
	// if the object is not enemy or if the self is not sheltered don't need to change shelter location
	if (objIdx > m_enemyVec.size() | scaledState[0] != scaledState[shelterIdx])
		return;

	// assumption : 1 shelter
	// if the enemy is closer to shelter move it to the new enemy location
	
	int distSelf = Distance(beliefState[0], beliefState[shelterIdx], m_gridSize);
	int distEnemy = Distance(beliefState[1], beliefState[shelterIdx], m_gridSize);
	if (distEnemy < distSelf)
	{
		scaledState[shelterIdx] = scaledState[1];
	}
}

void nxnGrid::ShiftSelfFromTarget(const intVec & beliefState, intVec & scaledState, int gridSize) const
{
	Coordinate realSelf(beliefState[0] % m_gridSize, beliefState[0] / m_gridSize);
	
	// if x > y shift on y axis else shift object on x axis
	if (realSelf.X() > realSelf.Y())
	{
		for (int i = 0; i < scaledState.size(); ++i)
		{
			int yLoc = scaledState[i] / gridSize - 1;

			// if the object is out of grid drop him from calculation
			if (yLoc < 0)
			{
				scaledState[i] = gridSize * gridSize;
			}
			else
			{
				scaledState[i] -= gridSize;
			}
		}
	}
	else
	{
		for (int i = 0; i < scaledState.size(); ++i)
		{
			int xLoc = scaledState[i] % gridSize - 1;

			// if the object is out of grid drop him from calculation
			if (xLoc < 0)
			{
				scaledState[i] = gridSize * gridSize;
			}
			else
			{
				--scaledState[i];
			}
		}
	}
}

void nxnGrid::DropUnProtectedShelter(intVec & state, int gridSize) const
{
	bool isProtected = false;
	int shelterIdx = 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
	for (int i = 0; i < shelterIdx; ++i)
	{
		isProtected |= state[i] == state[shelterIdx];
	}

	if (!isProtected)
		state[shelterIdx] = gridSize * gridSize;
}

inline bool nxnGrid::InBoundary(int location, int xChange, int yChange) const
{
	int x = location % m_gridSize + xChange;
	int y = location / m_gridSize + yChange;
	// return true if the location is in boundary
	return x >= 0 & x < m_gridSize & y >= 0 & y < m_gridSize;
}


enum nxnGrid::OBJECT nxnGrid::WhoAmI(int objIdx) const
{
	return objIdx == 0 ? SELF :
		objIdx < m_enemyVec.size() + 1 ? ENEMY :
		objIdx < m_nonInvolvedVec.size() + m_enemyVec.size() + 1 ? NON_INV :
		objIdx < m_shelters.size() + m_nonInvolvedVec.size() + m_enemyVec.size() + 1 ? SHELTER : TARGET;
}

void nxnGrid::InsertParticlesRec(intVec & state, std::vector<State*> & particles, double pToBelief, int currIdx) const
{
	if (state.size() == currIdx)
	{
		// insert belief state to vector
		nxnGridState* beliefState = static_cast<nxnGridState*>( Allocate(nxnGridState::StateToIdx(state), pToBelief) );
		particles.push_back(beliefState);
	}
	else
	{
		// diverge probability equally between all possible object location
		pToBelief /= m_gridSize * m_gridSize - currIdx;
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			state[currIdx] = i;
			if (NoRepetitions(state, currIdx, m_gridSize))
			{
				InsertParticlesRec(state, particles, pToBelief, currIdx + 1);
			}
		}
	}

}

int nxnGrid::FindObservation(intVec state, double p) const
{
	intVec obsState(CountMovingObjects());
	obsState[0] = state[0];
	// calculate observed state
	double rem = p;
	DecreasePObsRec(obsState, state, 1, 1.0, p);

	OBS_TYPE obs = nxnGridState::StateToIdx(obsState);
	nxnGridState s(nxnGridState::StateToIdx(state));

	// return the observed state
	return nxnGridState::StateToIdx(obsState);
}

void nxnGrid::SetNextPosition(intVec & state, std::vector<double> & randomNum) const
{
	// run on all enemies
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		// if the object is not dead calc movement
		if (state[i + 1] != m_gridSize * m_gridSize)
			CalcMovement(state, &m_enemyVec[i], randomNum[i], i + 1);
	}
	// run on all non-involved non-involved death is the end of game so don't need to check for death
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		// if the object is not dead calc movement
		if (state[i + 1 + m_enemyVec.size()] != m_gridSize * m_gridSize)
			CalcMovement(state, &m_nonInvolvedVec[i], randomNum[i + m_enemyVec.size()], i + 1 + m_enemyVec.size());
	}
}

void nxnGrid::CalcMovement(intVec & state, const Movable_Obj *object, double rand, int objIdx) const
{
	// if the p that was pulled is in the pStay range do nothing
	double pStay = object->GetMovement().GetStay();
	if (rand < pStay)
		return;

	// if the p that was pulled is in the pToward range move toward robot
	double pToward = pStay + object->GetMovement().GetToward();
	if (rand < pToward)
	{
		const Attack_Obj *enemy = static_cast<const Attack_Obj *>(object);
		// if the object is enemy and is allready in range do nothing
		if (WhoAmI(objIdx) == ENEMY && InRange(state[0], state[objIdx], enemy->GetRange(), m_gridSize))
		{
			return;
		}

		GetCloser(state, objIdx, m_gridSize);
		return;
	}
	// else treat the probability left as the new whole and cmove according to probability
	double pEqual = (rand - pToward) / (1 - pToward);
	int newLoc = FindObjMove(state[objIdx], pEqual, m_gridSize);
	if (ValidLocation(state, newLoc))
		state[objIdx] = newLoc;
}

bool nxnGrid::ValidLocation(intVec & state, int location)
{
	for (auto v : state)
	{
		if (location == v)
		{
			return false;
		}
	}

	return true;
}

bool nxnGrid::ValidLegalLocation(intVec & state, Coordinate location, int end, int gridSize)
{
	if ((location.X() >= gridSize) | (location.X() < 0) | (location.Y() >= gridSize) | (location.Y() < 0))
		return false;

	int locationIdx = location.X() + location.Y() * gridSize;
	for (size_t i = 0; i < end; ++i)
	{
		if (locationIdx == state[i])
		{
			return false;
		}
	}

	return true;
}

void nxnGrid::RescaleBelief(const intVec & beliefState, intVec & scaledState, int newGridSize, int oldGridSize) const
{
	double scale = static_cast<double>(oldGridSize) / newGridSize;

	for (size_t i = 0; i < beliefState.size(); ++i)
	{
		Coordinate location(beliefState[i] % oldGridSize, beliefState[i] / oldGridSize);
		location /= scale;
		scaledState[i] = location.X() + location.Y() * newGridSize;
	}

	// if the enemy is in the same spot move enemy to the neares available location
	if (!NoRepetitions(scaledState, 1, newGridSize))
		MoveObjectLocation(beliefState, scaledState, 1, newGridSize);

	// if target is in self location shift map to the left or upper so self won't be in target location 
	if (scaledState[0] == newGridSize * newGridSize - 1)
		ShiftSelfFromTarget(beliefState, scaledState, newGridSize);

	// if the non-involved is in non-valid location don't count him
	if (m_nonInvolvedVec.size() > 0 && !NoRepetitions(scaledState, m_enemyVec.size() + 1, newGridSize))
		MoveObjectLocation(beliefState, scaledState, m_enemyVec.size() + 1, newGridSize);


}

bool nxnGrid::NoRepetitions(intVec & state, int currIdx, int gridSize)
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

const Move_Properties & nxnGrid::GetMovement(int objIdx)
{
	// return move properties considering the object idx
	return objIdx == 0 ? m_self.GetMovement() :
		objIdx < m_enemyVec.size() + 1 ? m_enemyVec[objIdx - 1].GetMovement() :
		m_nonInvolvedVec[objIdx - 1 - m_enemyVec.size()].GetMovement();
}


void nxnGrid::DecreasePObsRec(intVec & currState, intVec & originalState, int currIdx, double pToDecrease, double &pLeft) const
{
	if (currIdx == nxnGridState::s_sizeState)
	{
		// decrease observed state probability from the left probability
		pLeft -= pToDecrease;
	}
	else
	{
		// if object is dead we know it
		if (originalState[currIdx] == m_gridSize * m_gridSize)
		{
			currState[currIdx] = m_gridSize * m_gridSize;
			DecreasePObsRec(currState, originalState, currIdx + 1, pToDecrease, pLeft);
			return;
		}

		// if location is in range disparse probability
		if (InRange(originalState[0], originalState[currIdx], m_self.GetRangeObs(), m_gridSize))
		{
			currState[currIdx] = originalState[currIdx];
			DecreasePObsRec(currState, originalState, currIdx + 1, pToDecrease * m_self.GetPObs(), pLeft);
			
			double pError = pToDecrease * (1 - m_self.GetPObs());
			for (size_t i = 0; i < 8 && pLeft >= 0.0; ++i)
			{
				currState[currIdx] = originalState[currIdx];
				int change = s_lutDirections[i][0] + s_lutDirections[i][1] * m_gridSize;
				currState[currIdx] += InBoundary(currState[currIdx], s_lutDirections[i][0], s_lutDirections[i][1]) ? change : 0;

				DecreasePObsRec(currState, originalState, currIdx + 1, pError / 8, pLeft);
			}
		}
		else // if we didn't observe object we think it does not exist
		{
			currState[currIdx] = m_gridSize * m_gridSize;
			DecreasePObsRec(currState, originalState, currIdx + 1, pToDecrease, pLeft);
		}
	}
}

inline int nxnGrid::CountMovingObjects() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

std::vector<double> nxnGrid::CreateRandomVec(int size)
{
	std::vector<double> randomVec;
	// insert to a vector numbers between 0-1
	for (int i = 0; i < size; ++i)
	{
		double r = std::rand();
		randomVec.push_back( r / RAND_MAX);
	}

	return randomVec;
}

void nxnGrid::GetCloser(intVec & state, int objIdx, int gridSize) const
{
	int xSelf = state[0] % gridSize;
	int ySelf = state[0] / gridSize;
	int xObj = state[objIdx] % gridSize;
	int yObj = state[objIdx] / gridSize;

	int objLocation = state[objIdx];
	int move = objLocation;

	int xDiff = xSelf - xObj;
	int yDiff = ySelf - yObj;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * m_gridSize : 0;

	// insert to move the best valid option
	if (ValidLocation(state, objLocation + changeToInsertX))
		move += changeToInsertX;
	if (ValidLocation(state, objLocation + changeToInsertY))
		move += changeToInsertY;

	state[objIdx] = move;
}

int nxnGrid::FindObjMove(int currLocation, double random, int gridSize) const
{
	int x = currLocation % gridSize;
	int y = currLocation / gridSize;
	// change x and y according to random probability
	if (random > 0.5)
	{
		random = (random - 0.5) * 2;
		if (random > 0.5)
		{
			++x;
			if (random > 0.75)
				++y;
			else
				--y;
		}
		else
		{
			--x;
			if (random > 0.25)
				++y;
			else
				--y;
		}
	}
	else
	{
		random *= 2;
		if (random > 0.5)
		{
			if (random > 0.75)
				++y;
			else
				--y;
		}
		else
		{
			if (random > 0.25)
				++x;
			else
				--x;
		}
	}

	if ((x >= 0) & (x < gridSize) & (y >= 0) & (y < gridSize))
		return x + y * gridSize;

	return currLocation;
}

bool nxnGrid::IsNear(int location, int observedLocation, int gridSize)
{
	int xReal = location % gridSize;
	int yReal = location / gridSize;

	int xObserved = observedLocation % gridSize;
	int yObserved = observedLocation / gridSize;

	int xDiff = Abs(xReal - xObserved);
	int yDiff = Abs(yReal - yObserved);

	return (xDiff <= 1) & (yDiff <= 1);
}


void nxnGrid::RandInitState()
{
	intVec state(CountMovingObjects());

	state[0] = 0;
	Coordinate p(state[0] % m_gridSize, state[0] / m_gridSize);
	m_self.SetLocation(p);
	size_t i = 1;
	for (; i < m_enemyVec.size() + 1; ++i)
	{
		do
		state[i] = rand() % (m_gridSize * m_gridSize);
		while (!nxnGrid::NoRepetitions(state, i, m_gridSize));

		Coordinate pEn(state[i] % m_gridSize, state[i] / m_gridSize);
		m_enemyVec[i - 1].SetLocation(pEn);
	}

	for (; i < m_nonInvolvedVec.size() + m_enemyVec.size() + 1; ++i)
	{
		do
		state[i] = rand() % (m_gridSize * m_gridSize);
		while (!nxnGrid::NoRepetitions(state, i, m_gridSize));

		Coordinate pNI(state[i] % m_gridSize, state[i] / m_gridSize);
		m_nonInvolvedVec[i - 1 - m_enemyVec.size()].SetLocation(pNI);
	}

	intVec shelters(m_shelters.size());
	for (size_t s = 0; s < m_shelters.size(); ++s)
	{
		do
		shelters[s] = rand() % (m_gridSize * m_gridSize);
		while (!nxnGrid::NoRepetitions(shelters, s, m_gridSize));

		Coordinate pSh(shelters[s] % m_gridSize, shelters[s] / m_gridSize);
		m_shelters[s].SetLocation(pSh);
		nxnGridState::s_shelters[s].SetLocation(pSh);
	}

	m_targetIdx = m_gridSize * m_gridSize - 1;
	nxnGridState::s_targetIdx = m_targetIdx;
}

int nxnGrid::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expecteReward)
{
	const nxnGrid * model = static_cast<const nxnGrid *>(m);
	int action = 0;

	if (prior->history().Size() > 0)
	{
		intVec beliefState;

		model->InitBeliefState(beliefState, prior->history());

		if (s_calculationType == ALL)
		{
			std::pair<int, double> result = model->m_LUT->Find(beliefState);
			// if the global decision was successful return action
			if (result.first != NUM_ACTIONS)
			{
				expecteReward = result.second;
				return result.first;
			}
		}
		else if (s_calculationType == WO_NINV)
		{
			beliefState.erase(beliefState.begin() + 1 + model->m_enemyVec.size());
			std::pair<int, double> result = model->m_LUT->Find(beliefState);
			// if the decision was successful return action
			if (result.first != NUM_ACTIONS)
			{
				expecteReward = result.second;
				return result.first;
			}
		}
		else if (s_calculationType == JUST_ENEMY)
		{
			// delete non-involved and shelter (in case of more than 1 objects type need to change this erase)
			beliefState.erase(beliefState.begin() + 1 + model->m_enemyVec.size());
			beliefState.erase(beliefState.begin() + 1 + model->m_enemyVec.size());
			std::pair<int, double> result = model->m_LUT->Find(beliefState);
			// if the decision was successful return action
			if (result.first != NUM_ACTIONS)
			{
				expecteReward = result.second;
				return result.first;
			}
		}
		else if (s_calculationType == RESCALE_ALL)
		{
			// create scaled state
			intVec scaledState(beliefState.size());
			model->RescaleBelief(beliefState, scaledState, model->m_LUT->GetSarsopGridSize(0), model->m_gridSize);
			// drop shelter from scaled state
			intVec woShelter(scaledState);
			model->DropUnProtectedShelter(woShelter, model->m_LUT->GetSarsopGridSize(0));
			
			std::pair<int, double> result = model->m_LUT->Find(scaledState);
			// if the decision was successful return action
			if (result.first != NUM_ACTIONS)
			{
				expecteReward = result.second;
				return result.first;
			}
		}
		else if (s_calculationType == STUPID)
			return MOVE_FROM_ENEMY;
	}

	return action;
}

void nxnGrid::InitBeliefState(intVec & beliefState,const History & h) const
{
	beliefState.resize(CountMovingObjects() + m_shelters.size());
	int lastObs = h.Size() - 1;

	beliefState[0] = GetObsLoc(h.Observation(lastObs), 0);
	int obj = 1;
	for (; obj < CountMovingObjects() ; ++obj)
	{
		// if in the future we want to run on more than last observation need to change this loop
		for (int i = 0; i < 1 && lastObs - i >= 0; ++i)
		{
			OBS_TYPE obs = h.Observation(lastObs - i);
			if (InObsRange(obs, obj))
			{
				beliefState[obj] = GetObsLoc(obs, obj);
				continue;
			}	
		}
	}

	for (auto v : m_shelters)
	{
		beliefState[obj] = v.GetLocation().GetIdx(m_gridSize);
		++obj;
	}
}

} //end ns despot