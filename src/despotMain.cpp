#include <fstream>      // std::ofstream

/// tui class
#include "../include/despot/simple_tui.h"

/// models available
#include "nxnGridGlobalActions.h"
#include "nxnGridLocalActions.h"

/// for nxnGrid

// properties of objects
#include "Coordinate.h"
#include "Move_properties.h"
#include "Attacks.h"
#include "Observations.h"

// objects
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "Self_Obj.h"
#include "ObjInGrid.h"

using namespace despot;

// choose model to run
enum MODELS_AVAILABLE { NXN_LOCAL_ACTIONS, NXN_GLOBAL_ACTIONS };
static MODELS_AVAILABLE s_UsingModel = NXN_GLOBAL_ACTIONS;

static std::vector<std::string> s_LUTFILENAMES{  "10x10Grid1x0x1_LUT_POMDP.bin"}; // "5x5Grid2x0x1_LUT_POMDP.bin", 
static std::vector<int> s_LUT_GRIDSIZE{ 10};
static std::vector<nxnGrid::CALCULATION_TYPE> s_CALCTYPE{ nxnGrid::CALCULATION_TYPE::WO_NINV }; // , nxnGrid::CALCULATION_TYPE::ONE_ENEMY

static int s_onlineGridSize = 10;

void ReadOfflineLUT(std::string & lutFName, std::map<STATE_TYPE, std::vector<double>> &offlineLut);
void Run(int argc, char* argv[], std::string & outputFName, int numRuns);
void InitObjectsLocations(std::vector<std::vector<int>> & objVec, int gridSize);

Attack_Obj CreateEnemy(int x, int y, int gridSize);
Movable_Obj CreateNInv(int x, int y);
Self_Obj CreateSelf(int x, int y, int gridSize);
ObjInGrid CreateShelter(int x, int y);

/// solve nxn Grid problem
class NXNGrid : public SimpleTUI {
public:
	NXNGrid() {}

	DSPOMDP* InitializeModel(option::Option* options) override
	{
		// create nxnGrid problem

		// init static members
		nxnGridState::InitStatic();

		Self_Obj self = CreateSelf(0, 0, s_onlineGridSize);
		int targetLoc = s_onlineGridSize * s_onlineGridSize - 1;

		std::vector<std::vector<int>> objVec(5);
		InitObjectsLocations(objVec, s_onlineGridSize);

		nxnGrid *model;
		if (s_UsingModel == NXN_LOCAL_ACTIONS)
			model = new nxnGridLocalActions(s_onlineGridSize, targetLoc, self, objVec);
		else if (s_UsingModel == NXN_GLOBAL_ACTIONS)
			model = new nxnGridGlobalActions(s_onlineGridSize, targetLoc, self, objVec);
		else
		{
			std::cout << "model not recognized... exiting!!\n";
			exit(0);
		}
		// add objects to model

		model->AddObj(CreateEnemy(0, 0, s_onlineGridSize));
		//model->AddObj(CreateEnemy(0, 0, s_onlineGridSize));

		model->AddObj(CreateNInv(0, 0));
		//model->AddObj(CreateNInv(0, 0));

		model->AddObj(CreateShelter(0, 0));
		return model;
	}

	void InitializeDefaultParameters() override {}
};

int main(int argc, char* argv[]) 
{
	int numRuns = 15;
	srand(time(NULL));

	nxnGrid::InitUDP();

	for (int j = 0; j < s_LUTFILENAMES.size(); ++j)
	{
		// init lut
		std::map<STATE_TYPE, std::vector<double>> offlineLut;
		ReadOfflineLUT(s_LUTFILENAMES[j], offlineLut);
		nxnGrid::InitLUT(offlineLut, s_LUT_GRIDSIZE[j], nxnGrid::ONLINE ,s_CALCTYPE[j]);
		// create output file

		std::string outputFName(s_LUTFILENAMES[j]);
		// pop ".bin"
		outputFName.pop_back();
		outputFName.pop_back();
		outputFName.pop_back();
		outputFName.pop_back();

		//outputFName.append("_resultOffline.txt");
		outputFName.append("_result.txt");
		Run(argc, argv, outputFName, numRuns);
	}

	{
		std::string outputFName("naive_result.txt");
		std::map<STATE_TYPE, std::vector<double>> offlineLut;
		nxnGrid::InitLUT(offlineLut, 10);
		Run(argc, argv, outputFName, numRuns);
	}

	char c;
	std::cout << "result written succesfully. press any key to exit\n";
	std::cin >> c;
	return 0;
}

void ReadOfflineLUT(std::string & lutFName, std::map<STATE_TYPE, std::vector<double>> &offlineLut)
{
	std::ifstream readLut(lutFName, std::ios::in | std::ios::binary);
	if (readLut.fail())
	{
		std::cout << "failed open lut file for write\n\n\n";
		exit(1);
	}
	else
	{
		int size;
		readLut.read(reinterpret_cast<char *>(&size), sizeof(int));
		int numActions;
		readLut.read(reinterpret_cast<char *>(&numActions), sizeof(int));
		for (int i = 0; i < size; ++i)
		{
			int state;
			readLut.read(reinterpret_cast<char *>(&state), sizeof(int));
			
			std::vector<double> rewards(numActions);
			for (int a = 0; a < numActions; ++a)
			{
				readLut.read(reinterpret_cast<char *>(&rewards[a]), sizeof(double));
			}

			offlineLut[state] = rewards;
		}
		if (readLut.bad())
		{
			std::cout << "failed write lut\n\n\n";
			exit(1);
		}
		else
			std::cout << "lut written succesfuly\n\n\n";

		readLut.close();
	}
}

void Run(int argc, char* argv[], std::string & outputFName, int numRuns)
{
	remove(outputFName.c_str());
	std::ofstream output(outputFName.c_str(), std::ios::out);
	if (output.fail())
	{
		std::cerr << "failed open output file";
		exit(1);
	}

	// run model numRuns times
	output << "results for naive online. num runs = " << numRuns << "\n";
	for (size_t i = 0; i < numRuns; i++)
	{
		std::cout << "\n\n\trun #" << i << ":\n";
		output << "\n\n\trun #" << i << ":\n";
		NXNGrid().run(argc, argv, output);
		output.flush();
	}

	if (output.fail())
	{
		std::cerr << "error in write output file";
		exit(1);
	}
}

void InitObjectsLocations(std::vector<std::vector<int>> & objVec, int gridSize)
{
	int obj = 0;

	// insert self locations
	objVec[obj].emplace_back(0);
	++obj;

	// insert enemies locations
	std::vector<int> enemy1Loc{ 99, 98, 89, 88};
	objVec[obj] = enemy1Loc;
	++obj;

	//std::vector<int> enemy2Loc{ 26, 27, 36, 37 };
	//objVec[obj] = enemy2Loc;
	//++obj;

	std::vector<int> nonInv1Loc{ 55, 56, 65, 66 };
	objVec[obj] = nonInv1Loc;
	++obj;

	std::vector<int> shelter1Loc{ 62,63,72,73 };
	objVec[obj] = shelter1Loc;

	// insert enemies locations
	//std::vector<int> enemy1Loc{ 399, 398, 397, 396, 379, 378, 377, 376, 359, 358, 357, 356, 339, 338, 337, 336};
	//objVec[obj] = enemy1Loc;
	//++obj;

	//std::vector<int> enemy2Loc{ 57 , 58, 77, 78 };
	//objVec[obj] = enemy2Loc;
	//++obj;

	//std::vector<int> nonInv1Loc{ 210 };
	//objVec[obj] = nonInv1Loc;
	//++obj;
	//
	//std::vector<int> nonInv2Loc{ 42};
	//objVec[obj] = nonInv2Loc;
	//++obj;

	//std::vector<int> shelter1Loc{244, 245, 246, 247, 264, 265, 266, 267, 284, 285, 286, 287, 304, 305, 306, 307, };
	//objVec[obj] = shelter1Loc;
}
Attack_Obj CreateEnemy(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;
	attackRange += (attackRange == 0);

	double pHit = 0.3;
	std::shared_ptr<Attack> attack(new DirectAttack(attackRange, pHit));

	double pStay = 0.4;
	double pTowardSelf = 0.4;

	Coordinate location(x, y);
	Move_Properties movement(pStay, pTowardSelf);
	return Attack_Obj(location, movement, attack);
}

Self_Obj CreateSelf(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;
	attackRange += (attackRange == 0);

	double pHit = 0.5;
	std::shared_ptr<Attack> attack(new DirectAttack(attackRange, pHit));
	double pDistanceFactor = 0.3;
	std::shared_ptr<Observation> obs(new ObservationByDistance(pDistanceFactor));

	double pMove = 0.9;
	double pStay = 1 - pMove;

	Coordinate location(x, y);
	Move_Properties movement(pStay, pMove);

	return Self_Obj(location, movement, attack, obs);
}

Movable_Obj CreateNInv(int x, int y)
{
	double pStay = 0.6;

	Coordinate location(x, y);
	Move_Properties movement(pStay);

	return Movable_Obj(location, movement);
}

ObjInGrid CreateShelter(int x, int y)
{
	double pStay = 0.6;

	Coordinate location(x, y);

	return ObjInGrid(location);
}
