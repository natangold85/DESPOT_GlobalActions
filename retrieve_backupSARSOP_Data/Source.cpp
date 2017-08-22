#include <iostream>      // std::out
#include <fstream>      // std::ofstream
#include <vector>      // std::ofstream
#include <cstring>      // strerror

/// for nxnGrid
#include "../src/MapOfPomdp.h"
#include "../src/POMDP_Writer.h"

const static char * DATAFILENAME = "data.bin";

using state_t = std::vector<int>;
using stateRewardMap = std::map<state_t, std::vector<double>>;

size_t FindNextUInt(char ** buffer)
{
	while (!isdigit(**buffer))
		++*buffer;
	size_t ret = atoi(*buffer);

	while (isdigit(**buffer))
		++*buffer;

	return ret;
}

double FindNextFloat(char ** buffer)
{
	while (!isdigit(**buffer))
		++*buffer;
	
	if (*(*buffer - 1) == '-')
		--*buffer;

	return strtod(*buffer, buffer);
}


void ReadKey(std::ifstream &in, MapOfPomdp::key_t & key)
{
	size_t mapLen = 1024;
	char *read = new char[mapLen + 1];
	read[0] = 0;
	char *buffer = read;

	while (!isdigit(*read))
		in.getline(buffer, mapLen);

	key[0] = FindNextUInt(&buffer);
	key[1] = FindNextUInt(&buffer);
	key[2] = FindNextUInt(&buffer);
	key[3] = FindNextUInt(&buffer);

	delete[] read;
}

void ReadOneCase(std::ifstream & in, MapOfPomdp & map)
{
	MapOfPomdp::key_t key(4);
	ReadKey(in, key);

	state_t state(1 + key[1] + key[2] + key[3]);
	char *ptr = new char[201];
	ptr[0] = 0;
	char *buffer = ptr;
	stateRewardMap pairs;

	while (!in.eof())
	{
		buffer = ptr;
		in.getline(buffer, 200);
		
		if (buffer[0] == 'm')
			break;

		while (!isdigit(*buffer))
			in.getline(buffer, 200);

		for (size_t i = 0; i < state.size(); ++i)
		{
			state[i] = FindNextUInt(&buffer);
		}

		std::vector<double> reward;
		
		reward.push_back(FindNextFloat(&buffer));
		reward.push_back(FindNextFloat(&buffer));
		reward.push_back(FindNextFloat(&buffer));
		reward.push_back(FindNextFloat(&buffer));
		pairs[state] = reward;
	}
	
	map.Add(key, pairs);
	delete[] ptr;
}

void ReadMap(MapOfPomdp & map, const char *fname)
{
	std::ifstream in(fname, std::ios::in);
	if (in.fail())
	{
		char errmsg[100];
		strerror_s(errmsg, 100, errno);
		std::cout << "Error in open: " << errmsg <<"\n";
		return;
	}
	while (!in.eof())
		ReadOneCase(in, map);
}

int main()
{
	MapOfPomdp map;
	ReadMap(map, "backup3x3.txt");
	ReadMap(map, "backup4x4.txt");
	ReadMap(map, "backup5x5.txt");

	std::cout << map;
	std::cout << "write file?(y for yes)\n";
	char c;
	std::cin >> c;
	if (c == 'y')
	{
		std::remove(DATAFILENAME);
		std::ofstream writeFile(DATAFILENAME, std::ios::out | std::ios::binary);
		if (writeFile.fail())
		{
			char errmsg[100];
			strerror_s(errmsg, 100, errno);
			std::cout << "Error in open: " << errmsg;

		}
		else
		{
			writeFile << map;


			if (writeFile.bad())
			{
				char errmsg[100];
				strerror_s(errmsg, 100, errno);
				std::cout << "Error in write: " << errmsg;
			}
		}
	}
	std::cout << "read file?(y for yes)\n";
	std::cin >> c;
	if (c == 'y')
	{
		std::ifstream readFile(DATAFILENAME, std::ios::in | std::ios::binary);
		MapOfPomdp map2;
		readFile >> map2;
		std::cout << map2;
	}
	std::cout << "press any key to exit\n";
	std::cin >> c;

}
