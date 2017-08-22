#include <fstream>      // std::ofstream
#include "MapOfPomdp.h"

const int MapOfPomdp::NUMBERS_IN_KEY = 4;

inline void READ(std::ifstream & in, char *toRead, int size)
{
	if (!in.read(toRead, size))
	{
		std::cerr << "error in reading\n";
		exit(1);
	}
}

bool MapOfPomdp::Add(key_t & key, stateRewardMap & stateRewardMap)
{
	auto itr = m_map.emplace(key, stateRewardMap);
	// if key allready exist erase and write again
	if (itr.second == false)
	{
		m_map.erase(itr.first);
		itr = m_map.emplace(key, stateRewardMap);
	}
	return itr.second;
}

bool MapOfPomdp::Find(key_t key, intVec state, double & reward) const
{
	auto itr = m_map.find(key);
	
	if (itr == m_map.end())
		return false;
	
	auto case_itr = itr->second.find(state);
	if (case_itr == itr->second.end())
		return false;

	reward = case_itr->second[0];
	return true;
}

std::ostream & operator<<(std::ostream & o, const MapOfPomdp & map)
{
	o << "number of pomdp : " << map.m_map.size() << "\n\n";
	
	std::for_each(map.m_map.begin(), map.m_map.end(), [&o](MapOfPomdp::PomdpMap::const_reference itr)
		{
		o << "map with : (gridSize, numEnemies, numNon-Involved, numShelters)\n";
			for (auto v: itr.first)
				o << v << ", ";

			o << "\n\n";
			std::for_each(itr.second.begin(), itr.second.end(), [&o](MapOfPomdp::stateRewardMap::const_reference itr) 
			{
				for (auto v : itr.first) 
					o << v << ", ";
				o << "Reward = " << itr.second[0] << " < " << itr.second[1] << ", " << itr.second[2] << "> Solver Duration = " << itr.second[3] <<"\n";
			});
		});

	return o;
}

std::ofstream & operator<<(std::ofstream & out, const MapOfPomdp & map)
{
	int size = map.m_map.size();
	
	out.write(reinterpret_cast<const char *>(&size), sizeof(int));

	std::for_each(map.m_map.begin(), map.m_map.end(), [&out](MapOfPomdp::PomdpMap::const_reference itrPomdp) 
	{
		out.write(reinterpret_cast<const char *>(&itrPomdp.first[0]), sizeof(int) * MapOfPomdp::NUMBERS_IN_KEY);

		int stateRewardSize = itrPomdp.second.size();
		out.write(reinterpret_cast<const char *>(&stateRewardSize), sizeof(int));

		std::for_each(itrPomdp.second.begin(), itrPomdp.second.end(), [&out](MapOfPomdp::stateRewardMap::const_reference itrStateReward)
		{
			out.write(reinterpret_cast<const char *>(&itrStateReward.first[0]), itrStateReward.first.size() * sizeof(int));
			out.write(reinterpret_cast<const char *>(&itrStateReward.second[0]), itrStateReward.second.size() * sizeof(double));
		});
	});

	return out;
}

std::ifstream & operator>>(std::ifstream & in, MapOfPomdp & map)
{
	int mapSize;
	in.read(reinterpret_cast<char *>(&mapSize), sizeof(int));

	for (int i = 0; i < mapSize; ++i)
	{
		// read pomdp
		MapOfPomdp::key_t key(MapOfPomdp::NUMBERS_IN_KEY);
		in.read(reinterpret_cast<char *>(&key[0]), sizeof(int) * MapOfPomdp::NUMBERS_IN_KEY);
		
		// read number of states for the current pomdp
		int stateSize;
		in.read(reinterpret_cast<char *>(&stateSize), sizeof(int));
		MapOfPomdp::stateRewardMap rewardMap;
		int numObj = key[1] + key[2] + 1; // enemies + non-involved + self
		MapOfPomdp::intVec state(numObj + key[3]); // numObj + numShelters
		// read pairs
		for (int i = 0; i < stateSize; ++i)
		{
			in.read(reinterpret_cast<char *>(&state[0]), state.size() * sizeof(int));
			std::vector<double> reward(4);
			in.read(reinterpret_cast<char *>(&reward[0]), sizeof(double) * 4);

			rewardMap.emplace(state, reward);

		}
		map.m_map.emplace(key, rewardMap);
	}
	return in;
}
