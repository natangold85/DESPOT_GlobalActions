#include <string>
#include <math.h>


#include "..\include\despot\solver\pomcp.h"
#include "nxnGrid.h"
#include "Coordinate.h"

namespace despot 
{

/// changes surrounding a point
static const int s_numMoves = 8;
static const int s_lutDirections[s_numMoves][2] = { { 0,1 },{ 0,-1 },{ 1,0 },{ -1,0 },{ 1,1 },{ 1,-1 },{ -1,1 },{ -1,-1 } };

/// second square changes surrounding the point
static const int s_numPixelsSecondSquare = 16;
static const int s_lutSecondSquare[s_numPixelsSecondSquare][2] = { { 0, 2 }, { 0, -2 }, { 2, 0 }, { -2, 0 },
																{ 1, 2 }, { 1, -2 }, { 2, 1 }, { -2, 1 },
																{ -1, 2 }, { -1, -2 }, { 2, -1 }, { -2, -1 },															
																{ 2, 2 }, { 2, -2 }, { -2, 2 }, { -2, -2 } };


// init static members

int nxnGridState::s_sizeState = 1;
int nxnGridState::s_numEnemies = 1;

int nxnGridState::s_gridSize = 0;
int nxnGridState::s_targetLoc = 0;

std::vector<int> nxnGridState::s_shelters;


int nxnGrid::s_numBasicActions = -1;
int nxnGrid::s_numEnemyRelatedActions = -1;

std::vector<nxnGrid::intVec> nxnGrid::s_objectsInitLocations;
UDP_Server nxnGrid::s_udpServer;

const double nxnGrid::REWARD_WIN = 50.0;
const double nxnGrid::REWARD_LOSS = -100.0;
const double nxnGrid::REWARD_KILL_ENEMY = 0.0;
const double nxnGrid::REWARD_KILL_NINV = REWARD_LOSS;
const double nxnGrid::REWARD_ILLEGAL_MOVE = 0;

// for lut
nxnGrid::lut_t nxnGrid::s_LUT;
int nxnGrid::s_lutGridSize;
enum nxnGrid::MODEL_TYPE nxnGrid::s_modelType = nxnGrid::ONLINE;
enum nxnGrid::CALCULATION_TYPE nxnGrid::s_calculationType = nxnGrid::WITHOUT;

// for synchronizing between sarsop rewards to despot rewards
static double REWARD_WIN_MAP = 1.0; 
static double REWARD_LOSS_MAP = -2.0;


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

nxnGridState::nxnGridState(STATE_TYPE state_id, double weight)
	: State(state_id, weight)
{
}

std::string nxnGridState::text() const
{
	intVec state;
	IdxToState(state_id, state);
	std::string ret = "(";
	for (auto v : state)
		ret += std::to_string(v) + ", ";
	ret += ")\n";
	for (int y = 0; y < s_gridSize; ++y)
	{
		for (int x = 0; x < s_gridSize; ++x)
		{
			int loc = x + y * s_gridSize;
			ret += ObjIdentity(state, loc);
		}
		ret += "\n";
	}

	return ret;
}

void nxnGridState::UpdateState(STATE_TYPE newStateIdx)
{
	state_id = newStateIdx;
}

void nxnGridState::UpdateState(intVec newState)
{
	state_id = StateToIdx(newState);
}

void nxnGridState::InitStatic()
{
	s_sizeState = 1;
	s_numEnemies = 0;
	s_gridSize = 0;
	s_shelters.resize(0);
}
void nxnGridState::IdxToState(STATE_TYPE idx, intVec & stateVec)
{
	stateVec.resize(s_sizeState);
	int numStates = s_gridSize * s_gridSize + 1;

	// running on all varied objects and concluding from the obs num the observed state
	for (int i = s_sizeState - 1; i >= 0; --i)
	{
		stateVec[i] = idx % numStates;
		idx /= numStates;
	}
}

STATE_TYPE nxnGridState::StateToIdx(const intVec &state)
{
	return StateToIdx(state, s_gridSize);
}

STATE_TYPE nxnGridState::StateToIdx(const intVec &state, int gridSize)
{
	STATE_TYPE idx = 0;
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

STATE_TYPE nxnGridState::MaxState()
{
	int numStates = s_gridSize * s_gridSize + 1;

	return pow(numStates, s_sizeState);
}

char nxnGridState::ObjIdentity(intVec & state, int location)
{
	if (state[0] == location)
		return 'M';

	int o = 1;
	
	for (; o < s_numEnemies + 1; ++o)
	{
		if (state[o] == location)
			return o + '0';
	}

	for (; o < state.size(); ++o)
	{
		if (state[o] == location)
			return 'N';
	}

	int s = 0;
	for (; s < s_shelters.size(); ++s)
	{
		if (location == s_shelters[s])
			return 'S';
	}

	if (location == s_targetLoc)
		return 'T';

	return '_';
}

/* =============================================================================
* nxnGrid Functions
* =============================================================================*/

nxnGrid::nxnGrid(int gridSize, int target, Self_Obj & self, std::vector<intVec> & objectsInitLoc)
	: m_gridSize(gridSize)
	, m_targetIdx(target)
	, m_self(self)
	, m_enemyVec()
	, m_shelters()
	, m_nonInvolvedVec()
{
	// init size  of state for nxnGridstate
	nxnGridState::s_sizeState = 1;
	nxnGridState::s_gridSize = gridSize;
	nxnGridState::s_targetLoc = target;

	s_objectsInitLocations = objectsInitLoc;
}

bool nxnGrid::InitUDP(int portNum)
{
	bool stat;
	if (portNum < 0)
		stat = s_udpServer.Init();
	else
		stat = s_udpServer.Init(portNum);

	std::cout << "udp connection initialized\n";

	return stat;
}

void nxnGrid::InitLUT(lut_t & offlineLut, int offlineGridSize, MODEL_TYPE mType,CALCULATION_TYPE ctype)
{
	s_modelType = mType;
	s_calculationType = ctype;
	s_lutGridSize = offlineGridSize;
	s_LUT = offlineLut;
}

void nxnGrid::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
	nxnGridState::s_sizeState = 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
	nxnGridState::s_numEnemies = m_enemyVec.size();

	AddActionsToEnemy();
}

void nxnGrid::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));
	nxnGridState::s_sizeState = 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

void nxnGrid::AddObj(ObjInGrid&& obj)
{
	m_shelters.emplace_back(std::forward<ObjInGrid>(obj));
	nxnGridState::s_shelters.emplace_back(obj.GetLocation().GetIdx(m_gridSize));
	
	AddActionsToShelter();
}



int nxnGrid::GetObsLoc(OBS_TYPE obs, int objIdx) const
{
	intVec obsState;
	nxnGridState::IdxToState(obs, obsState);

	return obsState[objIdx];
}

double nxnGrid::ObsProb(OBS_TYPE obs, const State & s, int action) const
{
	intVec state;
	nxnGridState::IdxToState(&s, state);

	intVec obsState;
	nxnGridState::IdxToState(obs, obsState);

	// if observation is not including the location of the robot return 0
	if (state[0] != obsState[0])
		return 0.0;

	double pObs = 1.0;
	// run on all non-self objects location
	for (int i = 1; i < CountMovingObjects(); ++i)
	{
		// create possible observation of obj location
		intVec observableLocations;
		m_self.GetObservation()->InitObsAvailableLocations(state[0], state[i], state, m_gridSize, observableLocations);
		
		// run on possible observable location
		bool isObserved = false;
		for (auto obsLoc : observableLocations)
		{
			if (obsLoc == obsState[i])
			{
				pObs *= m_self.GetObservation()->GetProbObservation(state[0], state[i], m_gridSize, obsLoc);
				isObserved = true;
			}
		}
		
		// if the object is not in observable locations return 0
		if (!isObserved)
			return 0.0;
	}
	
	return pObs;
}

double nxnGrid::ObsProbOneObj(OBS_TYPE obs, const State & s, int action, int objIdx) const
{
	intVec state;
	nxnGridState::IdxToState(&s, state);

	intVec obsState;
	nxnGridState::IdxToState(obs, obsState);

	if (objIdx == 0)
		return state[0] == obsState[0];

	double pObs = 0.0;

	// create possible observation of obj location
	intVec observableLocations;
	m_self.GetObservation()->InitObsAvailableLocations(state[0], state[objIdx], state, m_gridSize, observableLocations);

	// run on possible observable location
	for (auto obsLoc : observableLocations)
	{
		if (obsLoc == obsState[objIdx])
			return m_self.GetObservation()->GetProbObservation(state[0], state[objIdx], m_gridSize, obsLoc);
	}

	return 0.0;
}

void nxnGrid::CreateParticleVec(std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles) const
{
	intVec state(CountMovingObjects());
	CreateParticleVecRec(state, objLocations, particles, 1.0, 0);
}

void nxnGrid::ChoosePreferredActionIMP(intVec & beliefState, doubleVec & expectedReward) const
{
	lut_t::iterator itr, itr2;
	intVec scaledState;
	intVec modifiedBeliefState;

	switch (s_calculationType)
	{
	case ALL:
		scaledState.resize(beliefState.size());
		ScaleState(beliefState, scaledState);
		itr = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;

	case WO_NINV:
		// erase all non involved
		for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
			beliefState.erase(beliefState.begin() + 1 + m_enemyVec.size());

		scaledState.resize(beliefState.size());
		ScaleState(beliefState, scaledState);
		itr = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;

	case JUST_ENEMY:
		// erase all non involved
		for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
			beliefState.erase(beliefState.begin() + 1 + m_enemyVec.size());
		// erase all shelters
		for (int i = 0; i < m_shelters.size(); ++i)
			beliefState.erase(beliefState.begin() + 1 + m_enemyVec.size());

		scaledState.resize(beliefState.size());
		ScaleState(beliefState, scaledState);
		itr = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;

	case ONE_ENEMY:
		// erase all non involved
		for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
			beliefState.erase(beliefState.begin() + 1 + m_enemyVec.size());

		// calculate reward with first enemy
		modifiedBeliefState = beliefState;
		modifiedBeliefState.erase(modifiedBeliefState.begin() + 1 + 1);

		scaledState.resize(modifiedBeliefState.size());
		ScaleState(modifiedBeliefState, scaledState);
		itr = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));

		// calculate reward with second enemy
		modifiedBeliefState = beliefState;
		modifiedBeliefState.erase(modifiedBeliefState.begin() + 1);

		scaledState.resize(modifiedBeliefState.size());
		ScaleState(modifiedBeliefState, scaledState);
		itr2 = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));

		if (itr != s_LUT.end() & itr2 != s_LUT.end())
			Combine2EnemiesRewards(beliefState, itr->second, itr2->second, expectedReward);
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	
	case WO_NINV_STUPID:
		beliefState.erase(beliefState.begin() + 1 + m_enemyVec.size());
		scaledState.resize(beliefState.size());
		ScaleState(beliefState, scaledState);
		itr = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));
		
		// TODO : move members 1 slot right
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	default: // calc type = WITHOUT
		expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}
}

int nxnGrid::FindMaxReward(const doubleVec & rewards, double & expectedReward)
{
	int maxAction = -1;
	double maxReward = REWARD_LOSS - 1;

	for (int a = 0; a < rewards.size(); ++a)
	{
		if (rewards[a] > maxReward)
		{
			maxAction = a;
			maxReward = rewards[a];
		}
	}

	expectedReward = maxReward;
	return maxAction;
}

void nxnGrid::CreateParticleVecRec(intVec & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const
{
	if (currIdx == state.size())
	{
		auto beliefState = static_cast<nxnGridState*>(Allocate(nxnGridState::StateToIdx(state), weight));
		particles.push_back(beliefState);
	}
	else
	{
		for (auto loc : objLocations[currIdx])
		{
			state[currIdx] = loc.first;
			CreateParticleVecRec(state, objLocations, particles, weight * loc.second, currIdx + 1);
		}
	}
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
	int numStates = 1;
	// assumption : init states cannot be in same locations for different object
	for (int i = 1; i < CountMovingObjects(); ++i)
		numStates *= s_objectsInitLocations[i].size();

	double stateProb = 1.0 / numStates;
	
	InitialBeliefStateRec(state, 1, stateProb, particles);

	return new ParticleBelief(particles, this);
}

State * nxnGrid::Allocate(STATE_TYPE state_id, double weight) const
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
	out << state.text() << "\n";
}

void nxnGrid::PrintBelief(const Belief & belief, std::ostream & out) const
{
}

void nxnGrid::PrintObs(const State & state, OBS_TYPE obs, std::ostream & out) const
{
	nxnGridState obsState(obs);
	out << obsState.text() << "\n";
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
	
	m_enemyVec[enemyIdx].AttackOnline(state[enemyIdx + 1], state[0], state, shelters, m_gridSize, randomNum);

	return state[0] == m_gridSize * m_gridSize;
}

void nxnGrid::MoveNonProtectedShelters(const intVec & beliefState, intVec & scaledState, int newGridSize) const
{
	int startShelter = 1 + NumEnemiesInCalc() + NumNonInvInCalc();
	for (int s = 0; s < m_shelters.size(); ++s)
	{
		int shelterIdx = s + startShelter;
		
		for (int o = 0; o < startShelter; ++o)
		{
			if ((scaledState[o] == scaledState[shelterIdx]) & (beliefState[o] != beliefState[shelterIdx] & scaledState[o] != newGridSize * newGridSize))
			{
				MoveObjectLocation(beliefState, scaledState, o, newGridSize);
			}
		}
	}
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
	
	// TODO: see whats happenning to shelter after the moving of the object
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

	// if enemy is out of grid drop all other objects
	if (scaledState[1] == gridSize * gridSize)
	{
		for (int i = 2; i < scaledState.size(); ++i)
		{
			scaledState[i] = gridSize * gridSize;
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

int nxnGrid::NumEnemiesInCalc() const
{
	// TODO : make this function more modular
	return m_enemyVec.size() * (s_calculationType == WO_NINV) + 1 * (s_calculationType == ONE_ENEMY);
}

int nxnGrid::NumNonInvInCalc() const 
{
	// TODO : make this function more modular
	return 0;
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

void nxnGrid::InitialBeliefStateRec(intVec & state, int currObj, double stateProb, std::vector<State*> & particles) const
{
	if (currObj == CountMovingObjects())
	{
		auto beliefState = static_cast<nxnGridState*>(Allocate(nxnGridState::StateToIdx(state), stateProb));
		particles.push_back(beliefState);
	}
	else
	{
		for (auto obj : s_objectsInitLocations[currObj])
		{
			state[currObj] = obj;
			InitialBeliefStateRec(state, currObj + 1, stateProb, particles);
		}
	}
}
OBS_TYPE nxnGrid::FindObservation(intVec & state, double p) const
{
	intVec obsState(CountMovingObjects());
	obsState[0] = state[0];
	// calculate observed state
	DecreasePObsRec(obsState, state, 1, 1.0, p);

	// return the observed state
	return nxnGridState::StateToIdx(obsState);
}

void nxnGrid::SetNextPosition(intVec & state, doubleVec & randomNum) const
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
		// if the object is enemy and is allready in range do nothing
		if (WhoAmI(objIdx) == ENEMY)
		{
			const Attack_Obj *enemy = static_cast<const Attack_Obj *>(object);
			if (enemy->GetAttack()->InRange(state[0], state[objIdx], m_gridSize))
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

void nxnGrid::ScaleState(const intVec & beliefState, intVec & scaledState) const
{
	ScaleState(beliefState, scaledState, s_lutGridSize, m_gridSize);
}

void nxnGrid::ScaleState(const intVec & beliefState, intVec & scaledState, int newGridSize, int prevGridSize) const
{
	double scale = static_cast<double>(prevGridSize) / newGridSize;

	for (size_t i = 0; i < beliefState.size(); ++i)
	{
		Coordinate location(beliefState[i] % prevGridSize, beliefState[i] / prevGridSize);
		location /= scale;
		scaledState[i] = location.X() + location.Y() * newGridSize;
	}

	// if target is in self location shift map to the left or upper so self won't be in target location 
	if (scaledState[0] == newGridSize * newGridSize - 1)
		ShiftSelfFromTarget(beliefState, scaledState, newGridSize);

	
	int numEnemies = NumEnemiesInCalc();
	// run on enemies in calculation if the enemy is in the same spot move enemy to the nearest available location
	for (int i = 0; i < numEnemies; ++i)
	{
		if (!NoRepetitions(scaledState, i + 1, newGridSize))
			MoveObjectLocation(beliefState, scaledState, i + 1, newGridSize);
	}

	// run on non-involved in calculation if the non-involved is in non-valid location move him to the nearest location
	for (int i = 0; i < NumNonInvInCalc(); ++i)
	{
		if (!NoRepetitions(scaledState, numEnemies + 1 + i, newGridSize))
			MoveObjectLocation(beliefState, scaledState, m_enemyVec.size() + 1, newGridSize);
	}
	

	MoveNonProtectedShelters(beliefState, scaledState, newGridSize);
}

void nxnGrid::Combine2EnemiesRewards(const intVec & beliefState, const doubleVec & rewards1E, const doubleVec & rewards2E, doubleVec & rewards) const
{
	static int bitE1 = 1;
	static int bitE2 = 2;
	int deadEnemies = 0;

	rewards.resize(NumActions());
	// insert first enemy related actions rewards (if dead insert reward loss)
	if (beliefState[1] != m_gridSize * m_gridSize)
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a] = rewards1E[a];
	else
	{
		deadEnemies |= bitE1;
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a] = REWARD_LOSS;
	}
	// insert second enemy related actions rewards (if dead insert reward loss)
	if (beliefState[2] != m_gridSize * m_gridSize)
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a + s_numEnemyRelatedActions] = rewards2E[a];
	else
	{
		deadEnemies |= bitE2;
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a + s_numEnemyRelatedActions] = REWARD_LOSS;
	}

	// for non enemy related action do average of rewards
	for (int a = 0; a < s_numBasicActions; ++a)
	{
		// when only 1 enemy is dead insert to rewards calculation only reward of the live enemy
		double rE1 = rewards1E[a] * (deadEnemies != bitE1) + rewards2E[a] * (deadEnemies == bitE1);
		double rE2 = rewards2E[a] * (deadEnemies != bitE2) + rewards1E[a] * (deadEnemies == bitE2);
		rewards[a] = (rE1 + rE2) / 2;
	}
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


void nxnGrid::DecreasePObsRec(intVec & currState, const intVec & originalState, int currIdx, double pToDecrease, double &pLeft) const
{
	if (currIdx == CountMovingObjects())
	{
		// decrease observed state probability from the left probability
		pLeft -= pToDecrease;
	}
	else
	{
		int selfLoc = originalState[0];
		int objLoc = originalState[currIdx];
		intVec observableLocations;
		m_self.GetObservation()->InitObsAvailableLocations(selfLoc, objLoc, originalState, m_gridSize, observableLocations);
		
		for (int obs = 0; obs < observableLocations.size() & pLeft >= 0.0; ++obs)
		{
			currState[currIdx] = observableLocations[obs];
			double pObs = m_self.GetObservation()->GetProbObservation(selfLoc, objLoc, m_gridSize, observableLocations[obs]);
			DecreasePObsRec(currState, originalState, currIdx + 1, pToDecrease * pObs, pLeft);
		}
	}
}

inline int nxnGrid::CountMovingObjects() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

void nxnGrid::CreateRandomVec(doubleVec & randomVec, int size)
{
	// insert to a vector numbers between 0-1
	for (int i = 0; i < size; ++i)
	{
		randomVec.emplace_back(Random::RANDOM.NextDouble());
	}
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
	if (ValidLocation(state, move + changeToInsertX))
		move += changeToInsertX;
	if (ValidLocation(state, move + changeToInsertY))
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

bool nxnGrid::InSquare(int location, int location2, int squareSize,int gridSize)
{
	int xReal = location % gridSize;
	int yReal = location / gridSize;

	int xObserved = location2 % gridSize;
	int yObserved = location2 / gridSize;

	int xDiff = Abs(xReal - xObserved);
	int yDiff = Abs(yReal - yObserved);

	return (xDiff <= squareSize) & (yDiff <= squareSize);
}

void nxnGrid::SendAction(int action)
{
	s_udpServer.Write(reinterpret_cast<char *>(&action), sizeof(int));
}

bool nxnGrid::RcvState(State * s, int action, OBS_TYPE & obs, double & reward)
{
	intVec state(CountMovingObjects());
	intVec observation(CountMovingObjects());
	InitStateIMP(state, observation);

	if (state[0] == m_targetIdx)
	{
		SendAction(NumActions());
		return true;
	}
	
	s->state_id = nxnGridState::StateToIdx(state);
	obs = nxnGridState::StateToIdx(observation);

	reward = 0.0;
	if (state[0] == m_gridSize * m_gridSize)
		reward = REWARD_LOSS;
	else if (state[0] == m_targetIdx)
		reward = REWARD_WIN;
	//else if (EnemyRelatedAction(action) & (state[1] == m_gridSize * m_gridSize))
	//	reward = REWARD_ILLEGAL_MOVE;

	return false;
}

void nxnGrid::InitState()
{
	if (s_modelType == VBS)
		InitStateVBS();
	else
		InitStateRandom();
}

/// init state according to s_objectsInitLocations
void nxnGrid::InitStateRandom()
{
	int obj = 0;
	int idx = rand() % s_objectsInitLocations[obj].size();
	int loc = s_objectsInitLocations[obj][idx];
	m_self.SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
	++obj;

	for (int i = 0; i < m_enemyVec.size(); ++i, ++obj)
	{
		idx = rand() % s_objectsInitLocations[obj].size();
		loc = s_objectsInitLocations[obj][idx];
		m_enemyVec[i].SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
	}
	
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i, ++obj)
	{
		idx = rand() % s_objectsInitLocations[obj].size();
		loc = s_objectsInitLocations[obj][idx];
		m_nonInvolvedVec[i].SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
	}

	for (int i = 0; i < m_shelters.size(); ++i, ++obj)
	{
		idx = rand() % s_objectsInitLocations[obj].size();
		loc = s_objectsInitLocations[obj][idx];
		m_shelters[i].SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
		nxnGridState::s_shelters[i] = loc;
	}

}
/// init state from vbs simulator
void nxnGrid::InitStateVBS()
{
	intVec state(CountMovingObjects());
	intVec observation(CountMovingObjects());
	InitStateIMP(state, observation);

	int currObj = 0;
	Coordinate self(state[currObj] % m_gridSize, state[currObj] / m_gridSize);
	m_self.SetLocation(self);

	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		++currObj;
		Coordinate enemy;
		if (state[currObj] == m_gridSize * m_gridSize)
			enemy = Coordinate(m_gridSize, m_gridSize);
		else
			enemy = Coordinate(state[currObj] % m_gridSize, state[currObj] / m_gridSize);

		m_enemyVec[i].SetLocation(enemy);
	}

	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		++currObj;
		Coordinate ninv;
		if (state[currObj] == m_gridSize * m_gridSize)
			ninv = Coordinate(m_gridSize, m_gridSize);
		else
			ninv = Coordinate(state[currObj] % m_gridSize, state[currObj] / m_gridSize);

		m_nonInvolvedVec[i].SetLocation(ninv);
	}
}

void nxnGrid::InitStateIMP(intVec & state, intVec & observation)
{
	int buffer[40];
	int length = s_udpServer.Read(reinterpret_cast<char *>(buffer));

	int vbsGridSize = buffer[0];

	intVec identity;
	intVec readState;
	// read objects
	for (int obj = 2; obj < length / sizeof(int); obj += 2)
	{
		identity.emplace_back(buffer[obj - 1]);
		readState.emplace_back(buffer[obj]);
	}

	// organize objects according to identity
	intVec organizedState(readState.size());
	int obj = 0;
	organizedState[obj] = FindObject(readState, identity, SELF, 0);

	++obj;
	std::vector<bool> isObserved(organizedState.size());
	isObserved[0] = true;

	for (int i = 0; i < m_enemyVec.size(); ++i, ++obj)
	{
		bool isObs;
		organizedState[obj] = FindObject(readState, identity, ENEMY_VBS, OBSERVED_ENEMY_VBS, isObs, i);
		isObserved[obj] = isObs;
	}

	for (int i = 0; i < m_nonInvolvedVec.size(); ++i, ++obj)
	{
		bool isObs;
		organizedState[obj] = FindObject(readState, identity, NON_INVOLVED_VBS, OBSERVED_NON_INVOLVED_VBS, isObs, i);
		isObserved[obj] = isObs;
	}

	// read shelters
	for (int i = 0; i < m_shelters.size(); ++i, ++obj)
		organizedState[obj] = FindObject(readState, identity, SHELTER, i);


	// read and scale target
	int vbsTarget = FindObject(readState, identity, TARGET, 0);
	m_targetIdx = vbsTarget * (((double)(m_gridSize)) / vbsGridSize);
	nxnGridState::s_targetLoc = m_targetIdx;

	intVec scaledState(organizedState.size());
	ScaleState(organizedState, scaledState, m_gridSize, vbsGridSize);

	// insert to observation and state moving objects
	obj = 0;
	for (; obj < CountMovingObjects(); ++obj)
	{
		state[obj] = scaledState[obj];
		observation[obj] = state[obj] * isObserved[obj] + m_gridSize*m_gridSize * (!isObserved[obj]);
	}

	//insert scaled shelters location
	for (int i = 0; i < m_shelters.size(); ++i, ++obj)
	{
		nxnGridState::s_shelters[i] = scaledState[obj];
		m_shelters[i].SetLocation(Coordinate(scaledState[obj] % m_gridSize, scaledState[obj] / m_gridSize));
	}
}

int nxnGrid::FindObject(intVec & state, intVec & identity, int object, int idx)
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (identity[i] == object)
		{
			if (idx == 0)
				return state[i];

			--idx;
		}
	}

	return m_gridSize * m_gridSize;
}

int nxnGrid::FindObject(intVec & state, intVec & identity, int object, int observedObj, bool & isObserved, int idx)
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (identity[i] == object | identity[i] == observedObj)
		{
			if (idx == 0)
			{
				isObserved = identity[i] == observedObj;
				return state[i];
			}
			--idx;
		}
	}

	return m_gridSize * m_gridSize;
}

void nxnGrid::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards)
{
	const nxnGrid * model = static_cast<const nxnGrid *>(m);
	if (prior->history().Size() > 0)
	{
		intVec beliefState;
		model->InitBeliefState(beliefState, prior->history());
		model->ChoosePreferredActionIMP(beliefState, expectedRewards);
	}
	else
	{
		expectedRewards = doubleVec(model->NumActions(), REWARD_LOSS);
	}
}

int nxnGrid::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double expectedReward)
{	
	doubleVec rewards;
	ChoosePreferredAction(prior, m, rewards);
	// if calc type != without return lut result else return random decision
	if (s_calculationType != WITHOUT)
		return FindMaxReward(rewards, expectedReward);
	else
	{
		expectedReward = REWARD_LOSS;
		return rand() % m->NumActions();
	}
}

void nxnGrid::InitBeliefState(intVec & beliefState,const History & h) const
{
	beliefState.resize(CountMovingObjects());

	OBS_TYPE obs = h.LastObservation();
	beliefState[0] = GetObsLoc(obs, 0);
	
	intVec observedState;
	nxnGridState::IdxToState(obs, observedState);
	int obj = 1;
	for (; obj < CountMovingObjects() ; ++obj)
	{
		// if the object is non observed treat it as dead
		int loc = beliefState[0];
		// run on all history and initialize state with last location observed on object
		for (int o = h.Size() - 1; o >= 0 & loc == beliefState[0]; --o)
		{
			OBS_TYPE obs = h.Observation(o);
			loc = GetObsLoc(obs, obj);
		}

		beliefState[obj] = loc * (loc != beliefState[0]) + m_gridSize * m_gridSize * (loc == beliefState[0]);
	}

	AddSheltersLocations(beliefState);
}

void nxnGrid::AddSheltersLocations(intVec & state) const
{
	for (auto v : m_shelters)
		state.emplace_back(v.GetLocation().GetIdx(m_gridSize));
}

} //end ns despot