#include <iostream>		// cout, cin
#include <string>		// std::string
#include <Windows.h>	// CreateFile ..
#include <fstream>      // std::ofstream
#include <ctime>      // time
#include <algorithm>      // for_each


// possible model solutions possibilities
#include "../src/nxnGridOfflineGlobalActions.h"
#include "../src/nxnGridOfflineLocalActions.h"

// properties of objects
#include "../src/Coordinate.h"
#include "../src/Move_Properties.h"
#include "../src/Attacks.h"
#include "../src/Observations.h"

// objects
#include "../src/Self_Obj.h"
#include "../src/Attack_Obj.h"
#include "../src/Movable_Obj.h"
#include "../src/ObjInGrid.h"

// choose model to run
enum MODELS_AVAILABLE { NXN_LOCAL_ACTIONS, NXN_GLOBAL_ACTIONS };
static MODELS_AVAILABLE s_UsingModel = NXN_GLOBAL_ACTIONS;
// buffer size for read
const static int s_BUFFER_SIZE = 1024;

using lutSarsop = std::map<int, std::vector<double>>;

// appl solver directory location
std::string s_SOLVER_LOCATION = "D:\\madaan\\SARSOP\\src\\Release\\";

static std::ofstream s_BACKUP_FILE;

// lut from object identity to char symbolized it
static char * s_OBJ_TO_STRING;

void TranlateToX(const std::string &pomdp, const std::string &pomdpx)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	//si.dwFlags |= STARTF_USESTDHANDLES;
	ZeroMemory(&pi, sizeof(pi));

	//run solver with timeout of 100 seconds and precision boundary of 0.005 (the first among these two conditions)
	std::string cmd = s_SOLVER_LOCATION + "pomdpconvert.exe " + pomdp + " " + pomdpx;
	if (!CreateProcess(NULL, const_cast<char *>(cmd.c_str()), NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
	{
		std::cout << "CreateProcess failed (" << GetLastError() << ")\n";
		exit(1);
	}
	// wait for the solver to finish
	WaitForSingleObject(pi.hProcess, INFINITE);
}
/// run solver save policy in policty_name
void RunSolver(std::string &fname, std::string &policy_name, std::string &solverOutF)
{
	//STARTUPINFO si;
	//PROCESS_INFORMATION pi;

	//ZeroMemory(&si, sizeof(si));
	//si.cb = sizeof(si);
	//ZeroMemory(&pi, sizeof(pi));
	//// direct the output to the write pipe
	//si.hStdInput = NULL;

	//run solver with timeout of 10000 seconds and precision boundary of 0.005 (the first among these two conditions)
	std::string cmd = s_SOLVER_LOCATION + "pomdpsol.exe --timeout 30000 --precision 0.005 ";
	cmd += fname + " -o " + policy_name + " > " + solverOutF;
	
	//if (!CreateProcess(NULL, const_cast<char *>(cmd.c_str()), NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi))
	//{
	//	std::cerr << "CreateProcess failed (" << GetLastError() << ")\n";
	//	exit(1);
	//}

	//// wait for the evaluation to finish
	//WaitForSingleObject(pi.hProcess, INFINITE);
	
	
	int stat = system(cmd.c_str());

	if (stat != 0)
	{
		std::cerr << "error in running SARSOP solver\n";
		exit(1);
	}
}

void RunSimulator(std::string &fname, std::string &policy_name)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));
	// direct the output to the write pipe
	si.hStdInput = NULL;

	// run evaluator with 200 simulations
	std::string cmd = s_SOLVER_LOCATION + "pomdpsim.exe --simLen 50 --simNum 50 ";
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


void ReadReward(nxnGridOffline * pomdp, lutSarsop & sarsopMap, std::string & sarsopDataFName)
{
	std::ifstream data(sarsopDataFName, std::ios::in | std::ios::binary);
	if (data.fail())
	{
		std::cout << "open sarsop file failed with error code = " << GetLastError() << "\npress any key to exit\n";
		char c;
		std::cin >> c;
		exit(0);
	}

	int size;
	data.read((char *)&size, sizeof(int));
	int numActions;
	data.read((char *)&numActions, sizeof(int));
	for (int stateCount = 0; stateCount < size; ++stateCount)
	{
		int stateIdx = pomdp->StateCount2StateIdx(stateCount);
		sarsopMap[stateIdx].resize(numActions);

		for (int action = 0; action < numActions; ++action)
		{
			double reward;
			data.read((char *)&reward, sizeof(double));
			sarsopMap[stateIdx][action] = reward;
		}

	}
}

Attack_Obj CreateEnemy(int gridSize, Coordinate & location)
{
	double attackRange = static_cast<double>(gridSize) / 4;
	attackRange += (attackRange < 1);

	double pHit = 0.4;
	std::shared_ptr<Attack> attack(new DirectAttack(attackRange, pHit));

	double pStay = 0.35;
	double pTowardSelf = 0.4;

	Move_Properties movement(pStay, pTowardSelf);
	return Attack_Obj(location, movement, attack);
}

Self_Obj CreateSelf(int gridSize, Coordinate & location)
{
	double attackRange = static_cast<double>(gridSize) / 4;
	attackRange += (attackRange < 1);

	// for compensating the attack to non observed object
	double pHit = 0.5;
	std::shared_ptr<Attack> attack(new DirectAttack(attackRange, pHit));
	double pDistanceFactor = 0.4;
	std::shared_ptr<Observation> obs(new ObservationByDistance(pDistanceFactor));

	double pMove = 0.9;
	double pStay = 1 - pMove;

	Move_Properties movement(pStay, pMove);

	// location doesn't mind because each run we change location
	return Self_Obj(location, movement, attack, obs);
}

Movable_Obj CreateNInv(Coordinate & location)
{
	double pStay = 0.6;

	Move_Properties movement(pStay);

	// location doesn't mind because each run we change location
	return Movable_Obj(location, movement);
}

ObjInGrid CreateShelter(Coordinate & location)
{
	return ObjInGrid(location);
}

void CreateLUT(nxnGridOffline * pomdp, std::string & prefix, lutSarsop & sarsopMap)
{
	std::string pomdpFName = prefix + ".pomdp";
	// remove pomdp file before writing it
	remove(pomdpFName.c_str());
	FILE *fptr;
	auto err = fopen_s(&fptr, pomdpFName.c_str(), "w");
	if (0 != err)
	{
		std::cout << "ERROR OPEN\n";
		exit(1);
	}
	// write pomdp file
	pomdp->SaveInPomdpFormat(fptr);
	fclose(fptr);

	// print initial state
	std::cout << "finished writing pomdp file\n";

	// create policy and calculate duration of solver
	std::string policyName = prefix + ".policy";
	std::string solverOutFname = prefix + ".txt";
	time_t solverStart = time(nullptr);
	RunSolver(pomdpFName, policyName, solverOutFname);
	time_t solverDuration = time(nullptr) - solverStart;

	
	std::cout << "finished creating policy file\n";
	// run simulator
	RunSimulator(pomdpFName, policyName);
	std::cout << "finished running simulator\n";
	std::string sarsopDataFName = prefix + "_sarsopData.bin";
	ReadReward(pomdp, sarsopMap, sarsopDataFName);
}

void TraverseAllShelterLoc(nxnGridOffline * pomdp, std::vector<int> possibleShelterLoc, std::string & prefix, lutSarsop & sarsopMap)
{
	int gridSize = pomdp->GetGridSize();

	prefix += "S";
	for (auto s : possibleShelterLoc)
	{
		Coordinate point(s % gridSize, s / gridSize);
		pomdp->SetLocationShelter(point, pomdp->CountMovableObj());

		prefix += std::to_string(s);
		CreateLUT(pomdp, prefix, sarsopMap);
		prefix.pop_back();
		if (s > 9)
			prefix.pop_back();
		if (s > 99)
			prefix.pop_back();
	}
}

int main()
{
	// create model
	int gridSize = 10;
	
	Coordinate m(0, 0);
	int target = gridSize * gridSize - 1;

	nxnGridOffline * model = nullptr;

	if (s_UsingModel == NXN_LOCAL_ACTIONS)
		model = new nxnGridOfflineLocalActions(gridSize, target, CreateSelf(gridSize, m), false);
	else if (s_UsingModel == NXN_GLOBAL_ACTIONS)
		model = new nxnGridOfflineGlobalActions(gridSize, target, CreateSelf(gridSize, m), false);
	else
	{
		std::cout << "model not recognized... exiting!!\n";
		exit(0);
	}
	
	// ADD ENEMIES
	Coordinate e1(gridSize - 1, gridSize - 1);
	model->AddObj(CreateEnemy(gridSize, e1));

	//Coordinate e2(0, gridSize - 1);
	//model->AddObj(CreateEnemy(gridSize, e2));

	//ADD N_INV
	//Coordinate n(gridSize - 1, gridSize - 1);
	//model.AddObj(CreateNInv(n));

	//ADD SHELTERS
	Coordinate s(0, 0);
	model->AddObj(CreateShelter(s));

	// optional shelter loc
	std::vector<int> shelterLoc{ 62,63,72,73 };

	// create prefix for file name
	std::string prefix = std::to_string(gridSize) + "x" + std::to_string(gridSize) + "Grid";
	prefix += std::to_string(model->CountEnemies()) + "x" + std::to_string(model->CountNInv()) + "x" + std::to_string(model->CountShelters());

	std::string lutFName(prefix);
	lutFName += "_LUT.bin";
	lutSarsop sarsopMap;
	
	// create all format and solver options for the model
	//if (0 != model->CountShelters())
	//	TraverseAllShelterLoc(model, shelterLoc, prefix, sarsopMap);
	//else
	//	CreateLUT(model, prefix, sarsopMap);

	prefix += "S";
	for (auto s : shelterLoc)
	{
		Coordinate point(s % gridSize, s / gridSize);
		model->SetLocationShelter(point, model->CountMovableObj());
		std::string sarsopDataFName = prefix + std::to_string(s) + "_sarsopData.bin";
		ReadReward(model, sarsopMap, sarsopDataFName);
	}


	// write map to lut
	int numActions = model->GetNumActions();
	std::ofstream lut(lutFName, std::ios::out | std::ios::binary);
	int mapSize = sarsopMap.size();
	lut.write((const char *)&mapSize, sizeof(int));
	lut.write((const char *)&numActions, sizeof(int));
	std::for_each(sarsopMap.begin(), sarsopMap.end(), [&lut](lutSarsop::const_reference itr)
	{
		lut.write((const char *)&itr.first, sizeof(int));
		for (auto reward : itr.second)
		{
			lut.write((const char *)&reward, sizeof(double));
		}
	});

	std::string dest = "C:\\Users\\moshe\\Documents\\GitHub\\Despot\\" + lutFName;
	std::string src = "C:\\Users\\moshe\\Documents\\GitHub\\Despot\\" + lutFName;

	lut.close();
	if (!MoveFile(src.c_str(), dest.c_str()))
	{
		std::cout << "failed moving lut to main directory\n";
		std::cout << "press any key to exit";
		char c;
		std::cin >> c;
	}
}
