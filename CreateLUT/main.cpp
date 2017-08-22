#include "StateActionLUT.h"

#include <algorithm>

// lut file name
const char * LUTFILENAME = "Min_AllLUT.bin";
// sarsop data file name
const char * DATAFILENAME = "data.bin";

// key using lut
static std::vector<int> KEY{ 5,1,0,1 };

inline double Max(double a, double b)
{
	return a * (a >= b) + b * (a < b);
}

void NormalizeMapVals(std::map<int, double> & stateReward)
{
	double maxReward = -1000;
	// find max reward
	std::for_each(stateReward.begin(), stateReward.end(), [&maxReward](std::map<int, double>::const_reference itr) {maxReward = Max(maxReward, itr.second); });
	// divide all rewards by max reward
	std::for_each(stateReward.begin(), stateReward.end(), [maxReward](std::pair<const int, double> & pair) { pair.second /= maxReward; });
}

inline int ObjToKeyIdx(std::vector<int> & key, int objIdx)
{
	return objIdx <= key[1] ? 1 : objIdx <= key[1] + key[2] ? 2 : 3;
}

/// drop deads from key and state
void DropDeads(std::vector<int> & key, std::vector<int> & state)
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (state[i] == key[0] * key[0])
		{
			int keyIdx = ObjToKeyIdx(key, i);
			--key[keyIdx];
			state.erase(state.begin() + i);
			--i;
		}
	}
}

/// create lut[state_id] = offline reward
void CreateTmpEfficientSarsopLUT(std::vector<int> & key, std::vector<int> & state, int currIdx, std::map<int, double> & stateReward, MapOfPomdp & sarsopData)
{
	if (currIdx == state.size())
	{
		double reward;
		std::vector<int> forFindState(state);
		std::vector<int> forFindKey(key);
		DropDeads(forFindKey, forFindState);
		// if the data exist add him to map (for only one lut map default is to initialized to 0)
		if (sarsopData.Find(forFindKey, forFindState, reward))
			stateReward[StateActionLUT::State2Idx(state, key[0])] = reward;
	}
	else
	{
		for (size_t i = 0; i < key[0] * key[0]; ++i)
		{
			state[currIdx] = i;
			CreateTmpEfficientSarsopLUT(key, state, currIdx + 1, stateReward, sarsopData);
		}
		if (currIdx != 0)
		{
			state[currIdx] = key[0] * key[0];
			CreateTmpEfficientSarsopLUT(key, state, currIdx + 1, stateReward, sarsopData);
		}
	}
}

int Key2Key(int state_id, std::vector<int> & fromKey, std::vector<int> & toKey)
{
	int stateSize = 1 + fromKey[1] + fromKey[2] + fromKey[3];
	std::vector<int> state(StateActionLUT::Idx2State(state_id, stateSize, fromKey[0]));

	// assumption : maximum 1 member for each object
	if (fromKey[1] != toKey[1])
		state.insert(state.begin() + 1, toKey[0]);

	if (fromKey[2] != toKey[2])
		state.insert(state.begin() + 1 + toKey[1], toKey[0]);

	if (fromKey[3] != toKey[3])
		state.insert(state.begin() + 1 + toKey[1] + toKey[2], toKey[0]);

	return StateActionLUT::State2Idx(state, toKey[0]);
}

void AddStateReward(std::map<int, double> & stateReward, std::vector<int> & keyCurrent, std::map<int, double> & sumStateReward)
{
	std::for_each(stateReward.begin(), stateReward.end(), [&](std::map<int, double>::const_reference itr)
	{
		sumStateReward[Key2Key(itr.first, keyCurrent, KEY)] += itr.second;
	});
}

void CreateWeightedLUT(int gridSize, std::vector<std::vector<int>> keys, StateActionLUT::LUT_TYPE type)
{
	StateActionLUT lut(keys, gridSize, type);
	MapOfPomdp sarsopData;
	std::ifstream readFile(DATAFILENAME, std::ios::in | std::ios::binary);
	readFile >> sarsopData;
	
	std::vector<std::map<int, double>> stateRewardVec(keys.size());
	std::vector<DataForObjects> objects(keys.size());
	for (int i = 0; i < keys.size(); ++i)
	{
		std::vector<int> state(1 + keys[i][1] + keys[i][2] + keys[i][3]);
		
		// data on scaled objects
		double attackRange = static_cast<double>(KEY[0]) / 4;
		// to allow digonal shoot when gridsize = 5
		attackRange += 0.25 * (KEY[0] == 5);
		// to not allow less than 1 range
		attackRange = (attackRange < 1) ? 1 : attackRange;

		objects[i].m_moveProb = 0.9;
		objects[i].m_attackRange = attackRange;
		objects[i].m_attackRangeEnemy = attackRange;
		objects[i].m_observationRange = KEY[0] * 2;
		objects[i].m_pHit = 0.5;
		objects[i].m_pHitEnemy = 0.4;

		CreateTmpEfficientSarsopLUT(keys[i], state, 0, stateRewardVec[i], sarsopData);
		// normalize reward val to 1
		NormalizeMapVals(stateRewardVec[i]);
	}

	// create lut
	lut.InitLUTStateActions(&stateRewardVec, objects);

	// write lut to file
	std::remove(LUTFILENAME);
	std::ofstream writeLut(LUTFILENAME, std::ios::out | std::ios::binary);
	if (writeLut.fail())
	{
		std::cout << "failed open lut file for write\n";
	}
	else
	{
		writeLut << lut;
		if (writeLut.bad())
			std::cout << "failed write lut\n";
		else
			std::cout << "lut written succesfuly\n";

		writeLut.close();
	}
}

void CreateLUT(int gridSize)
{
	std::vector<std::vector<int>> keys;
	keys.emplace_back(KEY);
	StateActionLUT lut(keys, gridSize, StateActionLUT::LUT_TYPE::SINGLE);
	MapOfPomdp sarsopData;
	std::ifstream readFile(DATAFILENAME, std::ios::in | std::ios::binary);
	readFile >> sarsopData;

	std::vector<int> state(1 + KEY[1] + KEY[2] + KEY[3]);

	// data on scaled objects
	std::vector<DataForObjects> objects(1);

	double attackRange = static_cast<double>(KEY[0]) / 4;
	// to allow digonal shoot when gridsize = 5
	attackRange += 0.25 * (KEY[0] == 5);
	// to not allow less than 1 range
	attackRange = (attackRange < 1) ? 1 : attackRange;

	objects[0].m_moveProb = 0.9;
	objects[0].m_attackRange = attackRange;
	objects[0].m_attackRangeEnemy = attackRange;
	objects[0].m_observationRange = KEY[0] *2;
	objects[0].m_pHit = 0.5;
	objects[0].m_pHitEnemy = 0.4;

	std::vector<std::map<int, double>> stateReward(1);
	// create lut[state_id] = reward from offline table
	CreateTmpEfficientSarsopLUT(KEY, state, 0, stateReward[0], sarsopData);
	// normalize reward val to 1
	NormalizeMapVals(stateReward[0]);

	// create lut
	lut.InitLUTStateActions(&stateReward, objects);

	// write lut to file
	std::remove(LUTFILENAME);
	std::ofstream writeLut(LUTFILENAME, std::ios::out | std::ios::binary);
	if (writeLut.fail())
	{
		std::cout << "failed open lut file for write\n";
	}
	else
	{
		writeLut << lut;
		if (writeLut.bad())
			std::cout << "failed write lut\n";
		else
			std::cout << "lut written succesfuly\n";

		writeLut.close();
	}

}

int main()
{
	char c;
	using key_t = std::vector<int>;
	std::vector<key_t> keys;
	keys.emplace_back(key_t{ 4,1,0,1});
	keys.emplace_back(key_t{ 5,1,0,1 });
	keys.emplace_back(key_t{ 5,1,0,0 });
	

	CreateWeightedLUT(10, keys, StateActionLUT::LUT_TYPE::MIN);

	std::cout << "press any key to exit";
	std::cin >> c;
	return 0;
}