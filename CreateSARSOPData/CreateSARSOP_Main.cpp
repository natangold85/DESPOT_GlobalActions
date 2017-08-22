#include <iostream>		// cout, cin
#include <string>		// std::string
#include <Windows.h>	// CreateFile ..
#include <fstream>      // std::ofstream
#include <ctime>      // std::ofstream
#include <algorithm>      // for_each

#include "../src/MapOfPomdp.h"
#include "../src/POMDP_Writer.h"
#include "../src/Point.h"
#include "../src/Move_Properties.h"
#include "../src/Self_Obj.h"
#include "../src/Attack_Obj.h"
#include "../src/Movable_Obj.h"
#include "../src/ObjInGrid.h"

using identityVec = std::vector<POMDP_Writer::IDENTITY>;

const static int BUFFER_SIZE = 1024;

const static char * DATAFILENAME = "data.bin";
const static char * TMPFILENAME = "backup.bin";
std::string SOLVER_LOCATION = "C:\\Users\\natango\\Documents\\GitHub\\pomdp_solver\\src\\Release";

std::ofstream g_tmpBackupFile;

HANDLE g_writePipe, g_readPipe;

char * OBJECT_TO_STRING;

void InitPipe(HANDLE *writePipe, HANDLE *readPipe)
{
	SECURITY_ATTRIBUTES sa;
	ZeroMemory(&sa, sizeof(sa));
	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;

	if (!CreatePipe(readPipe, writePipe, &sa, 0))
	{
		std::cerr << "CreatePipe failed (" << GetLastError() << ")\n";
		exit(1);
	}
}

void RunSolver(std::string &fname, std::string &policy_name)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	si.dwFlags |= STARTF_USESTDHANDLES;
	ZeroMemory(&pi, sizeof(pi));

	//run solver with timeout of 100 seconds
	std::string cmd = SOLVER_LOCATION + "\\pomdpsol.exe --timeout 100 --precision 0.005 ";
	cmd += fname + " -o " + policy_name;
	if (!CreateProcess(NULL, const_cast<char *>(cmd.c_str()) , NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
	{
		std::cout << "CreateProcess failed (" << GetLastError() << ")\n";
		exit(1);
	}
	// wait for the solver to finish
	WaitForSingleObject(pi.hProcess, INFINITE);
}

void RunEvaluator(std::string &fname, std::string &policy_name)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));
	// direct the output to the write pipe
	si.dwFlags |= STARTF_USESTDHANDLES;
	si.hStdInput = NULL;
	si.hStdOutput = g_writePipe;
	si.hStdError = g_writePipe;

	// run evaluator with 400 simulations
	std::string cmd = SOLVER_LOCATION + "\\pomdpeval.exe --simLen 50 --simNum 200 --output-file test.log ";
	cmd += "--policy-file " + policy_name + " " + fname;
	if (!CreateProcess(NULL, const_cast<char *>(cmd.c_str()), NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi))
	{
		std::cerr << "CreateProcess failed (" << GetLastError() << ")\n";
		exit(1);
	}
	// wait for the evaluation to finish
	WaitForSingleObject(pi.hProcess, INFINITE);
}

double FindAndAdvance(const char **str)
{
	// search for the nearest number and return it
	const char *curr = *str;
	while (!isdigit(*curr))
	{
		++curr;
	}

	// if the  number is negative don't forget the minus
	if (*(curr - 1) == '-')
	{
		--curr;
	}
	return strtod(curr, const_cast<char**>(str));
}


std::vector<double> ReadReward()
{
	DWORD redword;
	std::string output;
	// read from pipe output from evaluation
	do
	{
		char buf[BUFFER_SIZE + 1];
		
		if (!ReadFile(g_readPipe, buf, BUFFER_SIZE, &redword, 0))
		{
			std::cerr << "ReadFile failed (" << GetLastError() << ")\n";
		}

		buf[redword] = '\0';
		output += buf;
	} while (redword == BUFFER_SIZE);

	//default value in case of error in solver
	std::vector<double> retval{ -1000.0, -1000.0, -1000.0 };
	// searching for the reward (according to the format of the evaluator) if the evaluator failed return retval filled with -1000
	auto found = output.find("Finishing ...");
	if (found == std::string::npos)
		return retval;

	const char *str = &(output.c_str()[found]);
	
	FindAndAdvance(&str);
	FindAndAdvance(&str);
	for (int i = 0; i < 3 ; ++i)
		retval[i] = FindAndAdvance(&str);

	return retval;
}

Attack_Obj CreateEnemy(int gridSize)
{
	double attackRange = static_cast<double>(gridSize) / 4;
	attackRange += (attackRange < 1);
	// for diagonal attack (need to overcome sqrt(2))
	attackRange += 0.25 * (gridSize == 5);

	double pHit = 0.4;
	double pStay = 0.4;
	double pTowardSelf = 0.4;

	Point location;
	Move_Properties movement(pStay, pTowardSelf);
	return Attack_Obj(location, movement, attackRange, pHit);
}

Self_Obj CreateSelf(int gridSize)
{
	double attackRange = static_cast<double>(gridSize) / 4;
	attackRange += (attackRange < 1);
	// for diagonal attack (need to overcome sqrt(2))
	attackRange += 0.25 * (gridSize == 5);

	double pHit = 0.5;
	int observationRange = gridSize * 2;
	double pSuccessObs = 0.9;

	
	double pMove = 0.9;
	double pStay = 1 - pMove;

	Point location;
	Move_Properties movement(pStay, pMove);

	return Self_Obj(location, movement, attackRange, pHit, observationRange, pSuccessObs);
}

Movable_Obj CreateNInv()
{
	double pStay = 0.6;

	Point location;
	Move_Properties movement(pStay);

	return Movable_Obj(location, movement);
}

ObjInGrid CreateShelter()
{
	Point location;

	return ObjInGrid(location);
}

bool NoRepeats(MapOfPomdp::intVec &state, int objIdx)
{
	for (int i = 0; i < objIdx; ++i)
	{
		if (state[i] == state[objIdx])
		{
			return false;
		}
	}
	return true;
}
void LastRec(POMDP_Writer & pomdp, MapOfPomdp::intVec & state, MapOfPomdp::stateRewardMap & stateMap, std::string & prefix)
{
	std::string fname = prefix + ".pomdp";
	// remove pomdp file before writing it
	remove(fname.c_str());
	FILE *fptr;
	auto err = fopen_s(&fptr, fname.c_str(), "w");
	if (0 != err)
	{
		std::cout << "ERROR OPEN\n";
		exit(1);
	}

	pomdp.SaveInFormat(fptr);
	fclose(fptr);
	std::cout << "for state: ";
	for (auto v : state)
		std::cout << v << ", ";

	// create policy 
	std::string policyName = prefix + ".policy";
	time_t solverStart = time(nullptr);
	RunSolver(fname, policyName);
	time_t solverDuration = time(nullptr) - solverStart;
	// find reward
	RunEvaluator(fname, policyName);

	std::vector<double> reward = ReadReward();
	reward.push_back(solverDuration);

	std::cout << "reward = " << reward[0] << " < " << reward[1] << ", " << reward[2] <<  ">  run time = "<< reward[3] << " ms\n\n";
	
	// insert pair to map and write pair to backup
	stateMap.emplace(state, reward);
	g_tmpBackupFile.write(reinterpret_cast<char *>(&state[0]), state.size() * sizeof(int));
	g_tmpBackupFile.write(reinterpret_cast<char *>(&reward[0]), reward.size() * sizeof(double));
}

void TraverseRec(POMDP_Writer & pomdp, MapOfPomdp::intVec & state, identityVec & identity, MapOfPomdp::stateRewardMap & stateMap, int objIdx, std::string & prefix)
{
	if (state.size() == objIdx)
	{
		// if the state is not already exist in the map insert it
		if (stateMap.find(state) == stateMap.end())
			LastRec(pomdp, state, stateMap, prefix);
		else
		{
			g_tmpBackupFile.write(reinterpret_cast<char *>(&state[0]), state.size() * sizeof(int));
			g_tmpBackupFile.write(reinterpret_cast<char *>(&stateMap[state][0]), stateMap[state].size() * sizeof(double));
		}

		return;
	}

	prefix.append(1, OBJECT_TO_STRING[identity[objIdx]]);;
	Point point;

	for (int i = 0; i < pomdp.GetGridSize() * pomdp.GetGridSize(); ++i)
	{
		state[objIdx] = i;
		point.SetIdx(i, pomdp.GetGridSize());

		if (NoRepeats(state, objIdx))
		{
			if (identity[objIdx] == POMDP_Writer::SELF)
				pomdp.SetLocationSelf(point);
			else if (identity[objIdx] == POMDP_Writer::ENEMY)
				pomdp.SetLocationEnemy(point, objIdx);
			else if (identity[objIdx] == POMDP_Writer::NON_INVOLVED)
				pomdp.SetLocationNonInv(point, objIdx);
			else
				pomdp.SetLocationShelter(point, objIdx);

			prefix += std::to_string(i);
			TraverseRec(pomdp, state, identity, stateMap, objIdx + 1, prefix);
			prefix.pop_back();
			if (i > 9)
				prefix.pop_back();
		}
		else if (identity[objIdx] == POMDP_Writer::SHELTER) //allow repetitions for shelters
		{
			pomdp.SetLocationShelter(point, objIdx);
			prefix += std::to_string(i);
			TraverseRec(pomdp, state, identity, stateMap, objIdx + 1, prefix);
			prefix.pop_back();
			if (i > 9)
				prefix.pop_back();
		}
	}
	prefix.pop_back();
}

void TraverseAllOptions(POMDP_Writer & pomdp, MapOfPomdp::intVec & state, identityVec & identity, MapOfPomdp::stateRewardMap & stateMap, std::string & prefix)
{
	int gridSize = pomdp.GetGridSize();

	// fixed target to be in the right corner
	int target = gridSize * gridSize - 1;
	pomdp.SetTarget(target);
	TraverseRec(pomdp, state, identity, stateMap, 0, prefix);
}

void ReadBackupToMap(std::ifstream & readBackupFile, MapOfPomdp::stateRewardMap & stateMap, int stateSize)
{
	// read state reward pairs and insert them to map (until eof)
	while (!readBackupFile.eof())
	{
		std::vector<int> state(stateSize);
		readBackupFile.read(reinterpret_cast<char *>(&state[0]), state.size() * sizeof(int));
		std::vector<double> reward(4);
		readBackupFile.read(reinterpret_cast<char *>(&reward[0]), reward.size() * sizeof(double));
		stateMap[state] = reward;
	}
}

void WriteMapToBackup(std::vector<int> & KeyToMap, MapOfPomdp::stateRewardMap & stateMapstateMap)
{
	g_tmpBackupFile.write(reinterpret_cast<char *>(&KeyToMap[0]), KeyToMap.size() * sizeof(int));

	std::for_each(stateMapstateMap.begin(), stateMapstateMap.end(), [](MapOfPomdp::stateRewardMap::const_reference itr)
	{
		g_tmpBackupFile.write(reinterpret_cast<const char *>(&itr.first[0]), itr.first.size() * sizeof(int));
		g_tmpBackupFile.write(reinterpret_cast<const char *>(&itr.second[0]), itr.second.size() * sizeof(int));
	});

}


int main()
{	
	InitPipe(&g_writePipe, &g_readPipe);

	// load map
	MapOfPomdp map;
	std::ifstream readFile(DATAFILENAME, std::ifstream::in | std::ifstream::binary);
	if (readFile.peek() != std::ifstream::traits_type::eof())
	{
		readFile >> map;
		if (readFile.bad())
		{
			std::cerr << "Error reading file";
			exit(1);
		}
		std::cout << map;
	}

	readFile.close();

	// translate type to file name
	OBJECT_TO_STRING = new char[4];
	OBJECT_TO_STRING[POMDP_Writer::SELF] = 'M';
	OBJECT_TO_STRING[POMDP_Writer::ENEMY] = 'E';
	OBJECT_TO_STRING[POMDP_Writer::NON_INVOLVED] = 'N';
	OBJECT_TO_STRING[POMDP_Writer::SHELTER] = 'S';

	// create model
	int gridSize = 5;
	// create identity vector for all objects
	identityVec identity;

	POMDP_Writer pomdp(gridSize, 0, CreateSelf(gridSize));
	identity.push_back(POMDP_Writer::SELF);

	// ADD ENEMIES
	pomdp.AddObj(CreateEnemy(gridSize));
	identity.push_back(POMDP_Writer::ENEMY);

	//ADD N_INV
	//pomdp.AddObj(CreateNInv());
	//identity.push_back(POMDP_Writer::NON_INVOLVED);

	//ADD SHELTERS
	pomdp.AddObj(CreateShelter());
	identity.push_back(POMDP_Writer::SHELTER);

	// key to map of solutions
	std::vector<int> KeyToMap(4);
	KeyToMap[0] = gridSize;
	KeyToMap[1] = pomdp.CountEnemies();
	KeyToMap[2] = pomdp.CountNInv();
	KeyToMap[3] = pomdp.CountShelters();
	
	MapOfPomdp::stateRewardMap stateMap;

	// retrieve backup data
	char a;
	std::cout << "\n\n use backup file? (y for yes)\n";
	std::cin >> a;
	if (a == 'y')
	{
		std::ifstream readBackupFile(TMPFILENAME, std::ios::in | std::ios::binary);
		// compare keys
		std::vector<int> backupKey(4);
		readBackupFile.read(reinterpret_cast<char *>(&backupKey[0]), backupKey.size() * sizeof(int));
		if (backupKey == KeyToMap)
		{
			// read state reward map
			ReadBackupToMap(readBackupFile, stateMap, 1 + backupKey[1] + backupKey[2] + backupKey[3]);
			readBackupFile.close();
		}
		else
		{
			std::cout << "map is not identitical to backup file(press any key to exit)";
			std::cin >> a;
			exit(1);
		}
	}

	std::remove(TMPFILENAME);
	g_tmpBackupFile.open(TMPFILENAME, std::ios::out | std::ios::binary);
	g_tmpBackupFile.write(reinterpret_cast<char *>(&KeyToMap[0]), KeyToMap.size() * sizeof(int));

	// create prefix for file name
	std::string prefix = std::to_string(gridSize) + "x" + std::to_string(gridSize) + "Grid";
	prefix += std::to_string(KeyToMap[1]) + "x" + std::to_string(KeyToMap[2]) + "x" + std::to_string(KeyToMap[3]);

	// create key for reward (state + targetIdx)
	MapOfPomdp::intVec state(pomdp.CountMovableObj() + pomdp.CountShelters());


	TraverseAllOptions(pomdp, state, identity, stateMap, prefix);
	delete[] OBJECT_TO_STRING;

	map.Add(KeyToMap, stateMap);

	std::cout << map;
	if (g_tmpBackupFile.bad())
	{
		std::cerr << "backup file was not written successfully";
	}
	g_tmpBackupFile.close();

	std::cout << "\n\nDo you want to save changes? (y for yes)\n";
	char c;
	std::cin >> c;
	if (c == 'y')
	{
		// remove file abefore writing the map
		std::remove(DATAFILENAME);
		std::ofstream writeFile(DATAFILENAME, std::ios::out | std::ios::binary);
		
		if (writeFile.fail())
			std::cerr << "Error opening file\n";
		else
		{
			writeFile << map;
			if (writeFile.bad())
				std::cerr << "Write map to file failed\n";
		}				

		std::cout << "\n\nMap saved successfuly\n";
		std::cin >> c;
	}

	CloseHandle(g_readPipe);
	CloseHandle(g_writePipe);

}
