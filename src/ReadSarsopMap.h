#pragma once
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

#include "../../write-pomdp-format/pomdp_writer/POMDP_Writer.h"

class ReadSarsopMap
{
public:
	using state_t = std::vector<int>;
	using stateRewardMap = std::map<state_t, std::vector<double>>;
	using PomdpMap = std::map<POMDP_Writer, stateRewardMap>;

	friend std::ostream& operator<<(std::ostream& o, const MapOfPomdp& map);

	friend std::ofstream& operator<<(std::ofstream& out, const MapOfPomdp& map);
	friend std::ifstream& operator>>(std::ifstream& in, MapOfPomdp& map);
private:
	PomdpMap m_map;
};



