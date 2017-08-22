#ifndef MAPOFPOMDP_H
#define MAPOFPOMDP_H

#include "POMDP_Writer.h"
#include <iostream>
#include <map>
#include <algorithm>


class MapOfPomdp
{
public:
	friend std::ostream& operator<<(std::ostream& o, const MapOfPomdp& map);
	friend std::ofstream& operator<<(std::ofstream& out, const MapOfPomdp& map);
	friend std::ifstream& operator>>(std::ifstream& in, MapOfPomdp& map);

	using key_t = std::vector<int>;
	using intVec = std::vector<int>;
	using stateRewardMap = std::map<intVec, std::vector<double>>;
	using PomdpMap = std::map<key_t, stateRewardMap>;

	enum KEY_IDX { GRIDSIZE = 0, NUM_ENEMIES = 1, NUM_NON_INVOLVED = 2, NUM_SHELTERS = 3 , KEYSIZE = 4};

	bool Add(key_t & key, stateRewardMap & stateRewardMap);
	bool Find(key_t key, intVec state, double & reward) const;

private:
	PomdpMap m_map;

	static const int NUMBERS_IN_KEY;
};


#endif	// MAPOFPOMDP_H