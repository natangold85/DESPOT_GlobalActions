#include "RockSampleSarsop.h"

#include <iostream>	// cerr
#include <fstream>	// iofstream
#include <string>	// string

std::vector<std::string> RockSampleSarsop::s_MOVES(NUM_MOVES);
std::vector<int> RockSampleSarsop::s_ADVANCE_FACTORS(NUM_MOVES + 1);

const double RockSampleSarsop::s_REWARD_NONVALID_MOVE = -100.0;
const double RockSampleSarsop::s_REWARD_GAMEOVER = 10.0;
const double RockSampleSarsop::s_REWARD_GOOD_SAMPLE = 10.0;
const double RockSampleSarsop::s_REWARD_BAD_SAMPLE = -10.0;
const double RockSampleSarsop::s_REWARD_BAD_LOCATION_SAMPLE = -100.0;

std::string s_gameOver("Game_Over");

inline double Distance(int loc1, int loc2, int gridSize)
{
	int xDiff = loc1 % gridSize - loc2 % gridSize;
	int yDiff = loc1 / gridSize - loc2 / gridSize;

	return sqrt(xDiff * xDiff + yDiff * yDiff);
}

RockSampleSarsop::RockSampleSarsop(int gridSize, rockVec rocks, double distanceRatio, double halfEfficiencyDistance, double discount)
: m_gridSize(gridSize)
, m_rocks(rocks)
, m_distanceRatio(distanceRatio)
, m_halfEfficiencyDistance(halfEfficiencyDistance)
, m_discount(discount)
{
	s_MOVES[NORTH] = "North";
	s_MOVES[SOUTH] = "South";
	s_MOVES[WEST] = "West";
	s_MOVES[EAST] = "East";

	s_ADVANCE_FACTORS[NORTH] = -gridSize;
	s_ADVANCE_FACTORS[SOUTH] = gridSize;
	s_ADVANCE_FACTORS[WEST] = -1;
	s_ADVANCE_FACTORS[EAST] = 1;
	s_ADVANCE_FACTORS[NUM_MOVES - 1] = 0;
}


void RockSampleSarsop::SaveInFormat(std::ofstream & file)
{
	//add comments and init lines(state observations etc.) to file
	CommentsAndInitLines(file);

	// add position with and without moving of the robot
	AddAllActions(file);

	// add observations and rewards
	AddObservations(file);

	AddRewards(file);
}

void RockSampleSarsop::CommentsAndInitLines(std::ofstream & file) const
{
	std::string buffer;

	// add comments
	buffer += "# pomdp file for rock sample problem:\n";
	buffer += "# grid size: " + std::to_string(m_gridSize);
	buffer += "\n# rocks (num rocks = " + std::to_string(m_rocks.size()) + "): ";
	for (auto v : m_rocks)
	{
		buffer += "\n# rock initial location: " + std::to_string(v.first) + "num Rocks = " + std::to_string(v.second.size()) + " status = ";
		for (auto status : v.second)
			buffer += RockStatus(status) + ", ";
	}

	// add init lines
	buffer += "\n\ndiscount: " + std::to_string(m_discount);
	buffer += "\nvalues: reward\nstates: ";

	// add states names
	InsertStates2Buffer(buffer);

	buffer += "actions: North South West East Sample";
	for (int i = 0; i < m_rocks.size(); ++i)
		buffer += " Check" + std::to_string(i);

	// add observations names
	buffer += "\nobservations: Bad Good None";
	buffer += "\n\nstart: \n";

	// add start states probability
	InsertStartState(buffer);
	buffer += "\n\n";
	//save to file
	file << buffer;
	if (file.bad()) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void RockSampleSarsop::AddAllActions(std::ofstream & file) const
{
	std::string buffer("");
	AddTransitionMoves(buffer);
	AddTransitionSample(buffer);
	AddTransitionCheck(buffer);

	//save to file
	file << buffer;
	if (file.bad()) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void RockSampleSarsop::AddObservations(std::ofstream & file) const
{
	std::string buffer("");
	// generalized all actions that retuns none observation
	buffer += "O: North: * : None 1\n";
	buffer += "O: South: * : None 1\n";
	buffer += "O: West: * : None 1\n";
	buffer += "O: East: * : None 1\n";
	buffer += "O: Sample: * : None 1\n\n";

	int numStates = NumStates();
	int numGood = NumOptionalGoodRocks();
	int goodIdx = 0;

	// run on all rocks for all check options
	for (int rock = 0; rock < m_rocks.size(); ++rock)
	{
		int rockLoc = m_rocks[rock].first;
		std::string action = "Check" + std::to_string(rock);
		//run on all states but game over
		for (int s = 0; s < numStates - 1; ++s)
		{
			// if the original rock status was good
			double efficiency = 0.0;
			if (IsGood(rock))
			{
				// if the current rock status is good
				if (s & (1 << goodIdx) > 0)
				{
					int selfLoc = s >> numGood;
					// calculate efficiency derived from despot program (multiply by ratio of online / offline grid)
					double distance = Distance(rockLoc, selfLoc, m_gridSize) * m_distanceRatio;
					efficiency = (1 + pow(2, -distance / m_halfEfficiencyDistance)) * 0.5;
					buffer += "\nO: " + action + " : " + State2String(s) + " : Good " + std::to_string(efficiency);
				}
				++goodIdx;
			}
			// insert prob to bad observation
			buffer += "\nO: " + action + " : " + State2String(s) + " : Bad " + std::to_string(1 - efficiency);
		}
		// insert none observation from gameover state
		buffer += "\nO: " + action + " : " + s_gameOver + " : None 1\n";
	}

	//save to file
	file << buffer;
	if (file.bad()) { std::cerr << "Error Writing to file\n"; exit(1); }
}


void RockSampleSarsop::AddRewards(std::ofstream & file) const
{
	std::string buffer("");
	int numGoodRocks = NumOptionalGoodRocks();
	// add reward for illegal moves(negative for all moves but east game over for east)
	RewardNonValidMove(buffer);
	
	// add negative reward for sample bad stone
	// add positive reward for sample good stone
	for (int s = 0; s < EndState(); ++s)
	{
		buffer += "\nR: Sample : " + State2String(s) + " : *: * ";
		int location = s >> numGoodRocks;
		int rockVecLoc = RockVecLoc(location);
		if (rockVecLoc != -1)
		{
			int rockIdx = InGoodRockPosition(s);
			if ( rockIdx != -1 & (s & (1 << rockIdx)) > 0)
				buffer += std::to_string(s_REWARD_GOOD_SAMPLE * CountGoodRocks(rockIdx));
			else
				buffer += std::to_string(s_REWARD_BAD_SAMPLE);
		}
		else
			buffer += std::to_string(s_REWARD_BAD_LOCATION_SAMPLE);
	}

	//save to file
	file << buffer;
	if (file.bad()) { std::cerr << "Error Writing to file\n"; exit(1); }
}
bool RockSampleSarsop::IsGood(int rockIdx) const
{
	bool good = false;
	for (auto v : m_rocks[rockIdx].second)
		good |= v;

	return good;
}

int RockSampleSarsop::RockVecLoc(int location) const
{
	for (int i = 0; i < m_rocks.size(); ++i)
	if (location == m_rocks[i].first)
		return i;
	
	return -1;
}

void RockSampleSarsop::InsertStates2Buffer(std::string & buffer) const
{
	int numGoodRocks = NumOptionalGoodRocks();
	int state = 0;
	// insert all states
	for ( ;state < m_gridSize * m_gridSize; ++state)
	{
		state <<= numGoodRocks;
		InsertRocks2BufferRec(state, numGoodRocks - 1, buffer);
		state >>= numGoodRocks;
	}

	// insert game over states
	buffer += " " + s_gameOver + "\n";;
}

void RockSampleSarsop::InsertRocks2BufferRec(int state, int currRock, std::string & buffer) const
{
	if (currRock < 0)
		buffer += "s" + std::to_string(state) + " ";
	else
	{
		// states where rock is bad
		state &= ~(1 << currRock);
		InsertRocks2BufferRec(state, currRock - 1, buffer);
		// states where rock is good
		state |= 1 << currRock;
		InsertRocks2BufferRec(state, currRock - 1, buffer);
	}
}

void RockSampleSarsop::InsertStartState(std::string & buffer) const
{
	// start state in the east middle location
	int startState = (m_gridSize / 2) * m_gridSize;
	int numGoodRocks = NumOptionalGoodRocks();

	for (int i = 0; i < numGoodRocks; ++i)
	{
		startState <<= 1;
		startState |= 1;
	}

	int numStates = NumStates();

	for (int i = 0; i < numStates; ++i)
	{
		if (i == startState)
			buffer += " 1";
		else
			buffer += " 0";
	}
}
int RockSampleSarsop::NumOptionalGoodRocks() const
{
	int countGood = 0;
	for (int i = 0; i < m_rocks.size(); ++i)
		countGood += IsGood(i);

	return countGood;
}

int RockSampleSarsop::NumStates() const
{
	// add 1 terminal state
	int numStates = m_gridSize * m_gridSize - 1;
	int numGoodRocks = NumOptionalGoodRocks();

	for (int i = 0; i < numGoodRocks; ++i)
	{
		numStates <<= 1;
		numStates |= 1;
	}
	// add 1 for state 0 and 1 for end state
	return numStates + 1 + 1;
}

void RockSampleSarsop::AddTransitionMoves(std::string & buffer) const
{
	int numGoodRocks = NumOptionalGoodRocks();
	int endState = EndState();

	for (int i = 0; i < m_gridSize * m_gridSize; ++i)
	{	
		// move north if the move is valid else new location is current location
		if (i / m_gridSize > 0)
			AddSingleMoveRec(i << numGoodRocks, NORTH, NORTH, numGoodRocks - 1, buffer);
		else
			AddSingleMoveRec(i << numGoodRocks, NORTH, NUM_MOVES, numGoodRocks - 1, buffer);
		buffer += "\n";
		
		// move south if the move is valid else new location is current location
		if (i / m_gridSize < m_gridSize - 1)
			AddSingleMoveRec(i << numGoodRocks, SOUTH, SOUTH, numGoodRocks - 1, buffer);
		else
			AddSingleMoveRec(i << numGoodRocks, SOUTH, NUM_MOVES, numGoodRocks - 1, buffer);
		buffer += "\n";

		// move west if the move is valid else new location is current location
		if (i % m_gridSize > 0)
			AddSingleMoveRec(i << numGoodRocks, WEST, WEST, numGoodRocks - 1, buffer);
		else
			AddSingleMoveRec(i << numGoodRocks, WEST, NUM_MOVES, numGoodRocks - 1, buffer);
		buffer += "\n";

		// move east if the move is valid else new location is current location
		if (i % m_gridSize < m_gridSize - 1)
			AddSingleMoveRec(i << numGoodRocks, EAST, EAST, numGoodRocks - 1, buffer);
		else
		{
			AddSingleMoveRec(i << numGoodRocks, EAST, NUM_MOVES, numGoodRocks - 1, buffer);
		}
		buffer += "\n";
	}
	for (int i = 0; i < NUM_MOVES; ++i)
		buffer += "T: " + s_MOVES[i] + " : " + s_gameOver + " : " + s_gameOver + " 1\n";
}

void RockSampleSarsop::AddSingleMoveRec(int state, enum MOVES move, enum MOVES actualMove, int currRock, std::string & buffer) const
{
	if (currRock < 0)
	{
		int newState;
		if (move == EAST & actualMove == NUM_MOVES)
			newState = EndState();
		else
			newState = state + (s_ADVANCE_FACTORS[actualMove] << NumOptionalGoodRocks());

		buffer += "T: " + s_MOVES[move] + " : " + State2String(state) + " : " + State2String(newState) + " 1\n";
	}
	else
	{
		int endState = EndState();
		// states where rock is bad
		state &= ~(1 << currRock);
		AddSingleMoveRec(state, move, actualMove, currRock - 1, buffer);
		// states where rock is good
		state |= 1 << currRock;
		AddSingleMoveRec(state, move, actualMove, currRock - 1, buffer);
	}
}

void RockSampleSarsop::AddTransitionSample(std::string & buffer) const
{
	int numStates = NumStates();
	for (int i = 0; i < numStates; ++i)
	{
		int idxRock = InGoodRockPosition(i);
		if (idxRock != -1)
		{
			int newState = i & ~(1 << idxRock);
			buffer += "T: Sample : " + State2String(i) + " : " + State2String(newState) + " 1\n";
		}
		else
			buffer += "T: Sample : " + State2String(i) + " : " + State2String(i) + " 1\n";
	}

	buffer += "\n";
}

int RockSampleSarsop::InGoodRockPosition(int state) const
{
	int numGoodRocks = NumOptionalGoodRocks();
	int location = state >> numGoodRocks;
	int goodIdx = 0;
	
	for (int i = 0; i < m_rocks.size(); ++i)
	{
		if (IsGood(i))
		{
			if (location == m_rocks[i].first & ((state & (1 << goodIdx)) > 0) )
			{
				return goodIdx;
			}
			++goodIdx;
		}
	}

	return -1;
}

int RockSampleSarsop::CountGoodRocks(int rockIdx) const
{
	int count = 0;
	for (auto v : m_rocks[rockIdx].second)
	{
		count += v;
	}

	return count;
}

void RockSampleSarsop::RewardNonValidMove(std::string & buffer) const
{
	int numGood = NumOptionalGoodRocks();
	// North
	for (size_t i = 0; i < m_gridSize; ++i)
		InsertRewardRec(i << numGood, numGood - 1, s_REWARD_NONVALID_MOVE, s_MOVES[NORTH], buffer);
	buffer += "\n";

	//South
	for (size_t i = m_gridSize * (m_gridSize - 1); i < m_gridSize * m_gridSize; ++i)
		InsertRewardRec(i << numGood, numGood - 1, s_REWARD_NONVALID_MOVE, s_MOVES[SOUTH], buffer);
	buffer += "\n";

	//West
	for (size_t i = 0; i < m_gridSize * m_gridSize; i += m_gridSize)
		InsertRewardRec(i << numGood, numGood - 1, s_REWARD_NONVALID_MOVE, s_MOVES[WEST], buffer);
	buffer += "\n";

	//East
	for (size_t i = m_gridSize - 1; i < m_gridSize * m_gridSize; i += m_gridSize)
		InsertRewardRec(i << numGood, numGood - 1, s_REWARD_GAMEOVER, s_MOVES[EAST], buffer);
	buffer += "\n";
}

void RockSampleSarsop::InsertRewardRec(int state, int currRock, double reward, std::string & action, std::string & buffer) const
{
	if (currRock < 0)
	{
		buffer += "\nR: " + action + " : " + State2String(state) + " : *: * " + std::to_string(reward);
	}
	else
	{
		// states where rock is bad
		state &= ~(1 << currRock);
		InsertRewardRec(state, currRock - 1, reward, action, buffer);
		// states where rock is good
		state |= 1 << currRock;
		// change newState rock flag unless its endState
		InsertRewardRec(state, currRock - 1, reward, action, buffer);
	}
}

void RockSampleSarsop::AddTransitionCheck(std::string & buffer) const
{
	int numStates = NumStates();
	for (int numRock = 0; numRock < m_rocks.size(); ++numRock)
	{
		for (int s = 0; s < numStates; ++s)
		{
			buffer += "T: Check" + std::to_string(numRock) + " : " + State2String(s) + " : " + State2String(s) + " 1\n";
		}
		buffer += "\n";
	}
}

std::string RockSampleSarsop::State2String(int state) const
{
	if (state != EndState())
		return "s" + std::to_string(state);
	else
		return s_gameOver;
}