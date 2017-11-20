#pragma once
#include <vector>

class RockSampleSarsop
{
public:
	using rockVec = std::vector<std::pair<int, std::vector<bool>>>;

	enum MOVES {NORTH, SOUTH, WEST, EAST, NUM_MOVES};

	/// rocks are vector of rocks with different locations (the bool vector is for if there is more than 1 rock in the same location) 
	/// distance ratio is (onlineGridSize / offline) 
	explicit RockSampleSarsop(int gridSize, rockVec rocks, double distanceRatio, double halfEfficiencyDistance = 20.0, double discount = 0.95);
	~RockSampleSarsop() = default;

	/// save model in pomdp format
	void SaveInFormat(std::ofstream & file);

private:
	int m_gridSize;
	rockVec m_rocks;
	double m_distanceRatio;
	/// parameter determining the range of observation
	double m_halfEfficiencyDistance; 
	double m_discount;

	/// lut from moves to its string
	static std::vector<std::string> s_MOVES; 
	/// lut from moves to its change in location on grid
	static std::vector<int> s_ADVANCE_FACTORS;

	/// rewards of model:
	static const double s_REWARD_NONVALID_MOVE;
	static const double s_REWARD_GAMEOVER;
	static const double s_REWARD_GOOD_SAMPLE;
	static const double s_REWARD_BAD_SAMPLE;
	static const double s_REWARD_BAD_LOCATION_SAMPLE;

	/// insert comments and init lines to file
	void CommentsAndInitLines(std::ofstream & file) const;
	/// insert transitions results of all actions
	void AddAllActions(std::ofstream & file) const;
	/// insert observations to file
	void AddObservations(std::ofstream & file) const;
	/// insert rewards to file
	void AddRewards(std::ofstream & file) const;

	/// insert list of all states to buffer
	void InsertStates2Buffer(std::string & buffer) const;
	/// run recursively on all rocks available status and to insert all available states to buffer
	void InsertRocks2BufferRec(int state, int currRock, std::string & buffer) const;

	/// insert initial state probability to buffer
	void InsertStartState(std::string & buffer) const;

	/// add transition in states with all move actions
	void AddTransitionMoves(std::string & buffer) const;
	/// add transition in states with sample action
	void AddTransitionSample(std::string & buffer) const;
	/// add transition in states with all check action
	void AddTransitionCheck(std::string & buffer) const;

	/// add single move to buffer given state (build rock status in function), move committed by robot and actual move been done by robot
	void AddSingleMoveRec(int state, enum MOVES move, enum MOVES actualMove, int currRock, std::string & buffer) const;
	
	/// insert reward for non valid moves (including negative reward and positive reward when the non-valid move is east to the end of the grid)
	void RewardNonValidMove(std::string & buffer) const;
	/// run on all rock status state and insert all states to buffer. given a location(state), reward and action
	void InsertRewardRec(int state, int currRock, double reward, std::string & action, std::string & buffer) const;

	/// return true if the rockIdx is good(at least one of the bool vector)
	bool IsGood(int rockIdx) const;
	/// return the rockIdx that is located in location. return -1 if there isn't any rock ;ocated in location
	int RockVecLoc(int location) const;
	/// TODO : understand what this function do
	int InGoodRockPosition(int state) const;
	/// return number of good rocks in a given rock idx
	int CountGoodRocks(int rockIdx) const;

	/// return status of a rock
	std::string RockStatus(bool status) const { return status ? "Good" : "Bad"; };
	/// return the name of a state
	std::string State2String(int oldState) const;
	/// return num optional good rocks
	int NumOptionalGoodRocks() const;
	/// return num states
	int NumStates() const;
	/// return end state
	int EndState() const { return NumStates() - 1; };
};

//  