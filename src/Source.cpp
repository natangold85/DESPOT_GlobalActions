#include <fstream>      // std::ofstream

/// tui class
#include "../include/despot/simple_tui.h"
#include "../include/despot/solver/pomcp.h"

/// models available
#include "nxnGridGlobalActions.h"

/// for nxnGrid
#include "Coordinate.h"
#include "Move_properties.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "Self_Obj.h"
#include "ObjInGrid.h"

using namespace despot;

static std::vector<std::string> s_LUTFILENAMES{  "10x10Grid1x0x1_LUT_POMDP.bin", "10x10Grid1x0x1_LUT_MDP.bin"};
static std::vector<int> s_LUT_GRIDSIZE{ 10, 10};
static std::vector<nxnGrid::CALCULATION_TYPE> s_CALCTYPE{ nxnGrid::CALCULATION_TYPE::WO_NINV, nxnGrid::CALCULATION_TYPE::WO_NINV };

int g_gridSize = 20;

Attack_Obj CreateEnemy(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;
	attackRange += (attackRange == 0);

	// multiply 0.5 by observation
	double pHit = 0.5 * 0.5;
	double pStay = 0.4;
	double pTowardSelf = 0.4;

	Coordinate location(x,y);
	Move_Properties movement(pStay, pTowardSelf);
	return Attack_Obj(location, movement, attackRange, pHit);
}

Self_Obj CreateSelf(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;
	attackRange += (attackRange == 0);

	double pHit = 0.5;

	int observationRange = 2 * gridSize;
	double pSuccessObs = 0.5;
	double pFirstSquare = 0.4;
	Self_Observation obs(observationRange, pSuccessObs, pFirstSquare);

	double pMove = 0.9;
	double pStay = 1 - pMove;

	Coordinate location(x, y);
	Move_Properties movement(pStay, pMove);

	return Self_Obj(location, movement, attackRange, pHit, obs);
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

/// solve nxn Grid problem
class NXNGrid : public SimpleTUI {
public:
	NXNGrid() {}

	DSPOMDP* InitializeModel(option::Option* options) 
	{
		// create nxnGrid problem

		// init static members
		nxnGridState::InitStatic();

		Self_Obj self = CreateSelf(0, 0, g_gridSize);

		nxnGridGlobalActions *model = new nxnGridGlobalActions(g_gridSize, 0, self);

		model->AddObj(CreateEnemy(0, 0, g_gridSize));
		model->AddObj(CreateNInv(0, 0));
		model->AddObj(CreateShelter(0, 0));

		return model;
	}

	void InitializeDefaultParameters() {
	}
};

void ReadOfflineLUT(std::string & lutFName, std::map<int, std::pair<int, double>> & offlineLut)
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
		for (int i = 0; i < size; ++i)
		{
			int state;
			readLut.read(reinterpret_cast<char *>(&state), sizeof(int));
			int action;
			readLut.read(reinterpret_cast<char *>(&action), sizeof(int));
			double reward;
			readLut.read(reinterpret_cast<char *>(&reward), sizeof(double));
			offlineLut[state] = std::make_pair(action, reward);
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

int main(int argc, char* argv[]) 
{
	int numRuns = 10;
	srand(time(NULL));

	for (int j = 0; j < s_LUTFILENAMES.size(); ++j)
	{
		// init lut
		std::map<int, std::pair<int, double>> offlineLut;
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
		std::map<int, std::pair<int, double>> offlineLut;
		nxnGrid::InitLUT(offlineLut, 10);
		Run(argc, argv, outputFName, numRuns);
	}

	char c;
	std::cout << "result written succesfully. press any key to exit\n";
	std::cin >> c;
	return 0;
}
